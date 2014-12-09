/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/**
 * Implementation of an quadrotor velocity control app for use in the 
 * semester project.
 */
 
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>

#include <systemlib/param/param.h>

#include <mavlink/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/quad_velocity_sp.h>
#include <uORB/topics/quad_pos_msg.h>
#include <uORB/topics/quad_mode.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "param.h"

// #define BUG(x) mavlink_log_info(mavlink_fd, "[quad_commander] Debug no. %i", x); /* to ease debug messages */

__EXPORT int quad_velocity_control_main(int argc, char *argv[]);
int quad_velocity_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

// Prototypes
// float velocity_controller()
void emergency(struct quad_velocity_sp_s *sp, struct quad_mode_s *mode, orb_advert_t *mode_pub);


static bool thread_running = false;
static bool thread_should_exit = false;
static int daemon_task;

int quad_velocity_control_thread_main(int argc, char *argv[]){
	warnx("[velocity_controller] started\n");

	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	struct quad_pos_msg_s quad_pos;
	memset(&quad_pos, 0, sizeof(quad_pos));
	struct quad_velocity_sp_s velocity_sp;
	memset(&velocity_sp, 0, sizeof(velocity_sp));
	struct quad_mode_s quad_mode;
	memset(&quad_mode, 0, sizeof(quad_mode));
	struct quad_alt_velocity state;
	memset(&state, 0, sizeof(state));
	
	struct quad_velocity_sp_s output;
	memset(&output, 0, sizeof(output));
	struct state_transition_s state_transition;
	memset(&state_transition, false, sizeof(state_transition));
	struct quad_alt_velocity sp;
	memset(&sp, 0, sizeof(sp));
	struct quad_alt_velocity error;
	memset(&error, 0, sizeof(error));


	// initialise take-off sequence
	struct init_pos_s takeoff_pos;
	memset(&takeoff_pos, 0, sizeof(takeoff_pos));

	int quad_pos_sub = orb_subscribe(ORB_ID(quad_pos_msg));
	int quad_mode_sub = orb_subscribe(ORB_ID(quad_mode));

	orb_advert_t quad_velocity_sp_pub = orb_advertise(ORB_ID(quad_velocity_sp), &velocity_sp);
	orb_advert_t quad_mode_pub = orb_advertise(ORB_ID(quad_mode), &quad_mode);

	float	Kp_thrust = 0.2,//0.00008,
	        Kd_thrust = 0.11, /* Controller constants for thrust controller */
	        Kp_pos = 0.06,
	        Kd_pos = 0.01, /* Controller constants for position controller */
	 	
	 	hover_alt = 1,		// 1 meter altitude setpoint
	 	landing_alt = 0.2,
		hover_threashold = 0.2,
		anti_gravity = 0.48,
		min_rotor_speed = 0.25,
		pos_max = 0.1,
		speed_up_time = 4,
		min_hover_velocity = 0.001,
		thrust_filter = 0.01,
		dt_pos = 0,
		time = 0,
	        time_old = hrt_absolute_time() / (float)1000000;

	const int system_id = 1;

	bool initialised = false,
	     shutdown_motors = true,
	     quad_mode_updated = false,
	     test = false;

	struct pollfd fds[1];
	fds[0].fd = quad_pos_sub;
	fds[0].events = POLLIN;

	while( !thread_should_exit ) {

		int pret = poll(fds, 1, 300);
		if (pret < 0) {
			mavlink_log_info(mavlink_fd,"[POT] Poll error");
		} else if (pret == 0){
			if (initialised){
				emergency(&velocity_sp, &quad_mode, &quad_mode_pub);
				mavlink_log_info(mavlink_fd,"[POT] Package loss limit reached");
			}

		} else if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(quad_pos_msg), quad_pos_sub, &quad_pos);

			orb_check(quad_mode_sub, &quad_mode_updated);
			
			if (quad_mode_updated){
				orb_copy(ORB_ID(quad_mode), quad_mode_sub, &quad_mode);
			}
			if (!initialised){
				time = hrt_absolute_time() / (float)1000000;
				dt_pos = time - time_old;
				time_old = time;
				initialised = true;
			} else {
				time = hrt_absolute_time() / (float)1000000;
	                        dt_pos = time - time_old;
	                        time_old = time;		
			}


			// Set state values
			state.x_old = state.x;
			state.y_old = state.y;
			state.z_old = state.z;

			state.x = quad_pos.x[system_id - 1] / (float)1000;
			state.y = quad_pos.y[system_id - 1] / (float)1000;
			state.z = quad_pos.z[system_id - 1] / (float)1000;

			state.dx = (state.x - state.x_old)/dt_pos;
			state.dy = (state.y - state.y_old)/dt_pos;
			state.dz = (state.z - state.z_old)/dt_pos;

			// Set initial values when received commands
			if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_TAKEOFF && !state_transition.takeoff){

				sp.timestamp = time;
				sp.x = state.x;
				sp.y = state.y;
				sp.z = hover_alt;

				state_transition.takeoff = true;
				shutdown_motors = false;
				mavlink_log_info(mavlink_fd,"[POT] TAKEOFF INITIALISED");

			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_LAND && !state_transition.land){
				// initialise landing sequence
				sp.x = state.x;
				sp.y = state.y;

				state_transition.land = true;
				mavlink_log_info(mavlink_fd,"[POT] LANDING INITIALISED");

			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_START_SWARM && !state_transition.start){
			
			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_STOP_SWARM && !state_transition.stop){
				
				// // Stop computing potential fields and break formation and return to hovering
				// struct init_pos_s landing_pos;
				// memset(&landing_pos, 0, sizeof(landing_pos));

				// landing_pos.timestamp = quad_pos.timestamp;
				// landing_pos.x = state.x;
				// landing_pos.y = state.y;
				// landing_pos.z = state.z;

				// state_transition = true;

			} else {
				// Do nothing yet
			}


			// Check for state shifts
			if (state_transition.takeoff){
				
				// Takeoff sequence
				if ((state.z > (sp.z - (float)hover_threashold)) && (state.z < (sp.z + (float)hover_threashold)) && ((float)fabs(state.dz) < min_hover_velocity)){
					
					// Change state to hovering state
					state_transition.takeoff = false;
					quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
					quad_mode.current_state = (enum QUAD_STATE)QUAD_STATE_HOVERING;
					orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
					mavlink_log_info(mavlink_fd,"[POT] HOVERING");
				} else {
					// Do nothing
				}

			} else if (state_transition.land){

				if(state.z > hover_alt ) {
					sp.z = state.z - (float)0.1;
				} else if (state.z < hover_alt && state.z > (float)0.5){
					sp.z = state.z - (float)0.02;		
				} else if (state.z < (float)0.5 && state.z > landing_alt){
					sp.z = state.z - (float)0.005;
				} else {
					
					shutdown_motors = true;
					state_transition.land = false;
					quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
					quad_mode.current_state = (enum QUAD_STATE)QUAD_STATE_GROUNDED;
                        		orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);

                        		mavlink_log_info(mavlink_fd,"[POT] LANDED");
				}

			} else if (state_transition.start){

			} else if (state_transition.stop){

			} else {

			}

                        if (shutdown_motors) {

                        	velocity_sp.thrust = 0;

                        } else if (!quad_mode.error && !shutdown_motors) {

				// Thrust controller
				error.thrust = sp.z - state.z;
				error.thrust_der = (error.thrust - error.thrust_old)/(float)dt_pos;

				error.thrust_old = error.thrust;
				state.thrust_old = output.thrust;

				output.thrust = Kp_thrust * error.thrust + Kd_thrust * error.thrust_der;

				// Thrust filter
	                        if (output.thrust > state.thrust_old + thrust_filter){
	                                output.thrust = state.thrust_old + thrust_filter;
	                        } else if (output.thrust < state.thrust_old - thrust_filter) {
	                                output.thrust = state.thrust_old - thrust_filter;
	                        }

	                        // Thrust limiter
	                        if ( output.thrust > (float)1 ) {
	                                output.thrust = (float)1;
	                        } else if ( output.thrust < 0 ) {
	                                output.thrust = 0;
	                        }

				velocity_sp.thrust = output.thrust + anti_gravity;

				if ((sp.timestamp + (float)speed_up_time) > time){
					velocity_sp.thrust = min_rotor_speed;
				}
                        } else {
                        	// Do nothing
                        }

                        // // Velocity controller
                        // error.dx = sp.dx - state.dx;
                        // error.dy = sp.dx - state.dy;

                        // // Derivative part
                        // error.ddx = (error.dx - error.dx_old)/(float)dt_pos;
                        // error.ddy = (error.dy - error.dy_old)/(float)dt_pos;

                        // error.dx_old = error.dx;
                        // error.dy_old = error.dy;

                        // // PD velocity controller
                        // output.pitch = - (float)Kp_pos * error.dx - (float)Kd_pos * error.ddx;
                        // output.roll  = - (float)Kp_pos * error.dy - (float)Kd_pos * error.ddy;

                        // Position controller
                        error.x = sp.x - state.x;
                        error.y = sp.y - state.y;

                        error.dx = (error.x - error.x_old)/(float)dt_pos;
                        error.dy = (error.y - error.y_old)/(float)dt_pos;

                        error.x_old = error.x;
                        error.y_old = error.y;

                        output.pitch = - Kp_pos * error.x - Kd_pos * error.dx;
                        output.roll  = - Kp_pos * error.y - Kd_pos * error.dy;


                        /* Limiting position controller output */
                        if ((float)fabs(output.roll) > pos_max)
                                output.roll = pos_max * (output.roll / (float)fabs(output.roll));

                        if ((float)fabs(output.pitch) > pos_max)
                                output.pitch = pos_max * output.pitch / (float)fabs(output.pitch);

                        velocity_sp.roll = output.roll;
                        velocity_sp.pitch = output.pitch;

                        // mavlink_log_info(mavlink_fd,"th: %.3f r: %.3f p: %.3f yaw: %.3f ",(double)velocity_sp.thrust,(double)velocity_sp.roll,(double)velocity_sp.pitch,(double)velocity_sp.yaw);
                        // Publish the new roll, pitch, yaw and thrust set points
                        
                        if(test) {
                        	velocity_sp.thrust = 0;
                        	velocity_sp.roll = 0;
                        	velocity_sp.pitch = 0;
         	                velocity_sp.yaw = 0;
                        }

                        orb_publish(ORB_ID(quad_velocity_sp), quad_velocity_sp_pub, &velocity_sp);
		}
	}
	return 0;
}

void emergency(struct quad_velocity_sp_s *sp, struct quad_mode_s *mode, orb_advert_t *mode_pub){
	sp->thrust = 0;
	sp->roll = 0;
	sp->pitch = 0;
	sp->yaw = 0;

	mode->error = true;
	mode->current_state = (enum QUAD_STATE)QUAD_STATE_EMERGENCY;

	orb_publish(ORB_ID(quad_mode), *mode_pub, mode);


}


// User interface for running in shell
static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: velocity_controller {start|stop|status}\n\n");
        exit(1);
}

int quad_velocity_control_main(int argc, char *argv[]) {
        if (argc < 1)
                usage("missing argument");

        if (!strcmp(argv[1], "start")) {

                if (thread_running) {
                        printf("velocity_controller already running\n");

                        exit(0);
                }

                thread_should_exit = false;
                daemon_task = task_spawn_cmd("velocity_controller",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 5,
                                             2048,
                                             quad_velocity_control_thread_main,
                                             (argv) ? (const char **)&argv[2] : (const char **)NULL);
                thread_running = true;
                exit(0);
        }

        if (!strcmp(argv[1], "stop")) {
                thread_should_exit = true;
                exit(0);
        }

        if (!strcmp(argv[1], "status")) {
                if (thread_running) {
                        printf("\tvelocity_controller is running\n");

                } else {
                        printf("\tvelocity_controller not started\n");
                }

                exit(0);
        }

        usage("unrecognized command");
        exit(1);
}
