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
#include "swarm_formation.h"


// #define BUG(x) mavlink_log_info(mavlink_fd, "[quad_commander] Debug no. %i", x); /* to ease debug messages */

__EXPORT int quad_velocity_control_main(int argc, char *argv[]);
int quad_velocity_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

// Prototypes
// float velocity_controller()
void emergency( struct quad_mode_s *mode, orb_advert_t *mode_pub );
void shutdown( struct quad_velocity_sp_s *sp, orb_advert_t *quad_velocity_sp_pub );


static bool thread_running = false;
static bool thread_should_exit = false;
static int daemon_task;

int quad_velocity_control_thread_main(int argc, char *argv[]) {
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

	struct vehicle_status_s vehicle_status;
	memset(&vehicle_status, 0, sizeof(vehicle_status));


	// initialise take-off sequence
	struct init_pos_s takeoff_pos;
	memset(&takeoff_pos, 0, sizeof(takeoff_pos));

	int quad_pos_sub = orb_subscribe(ORB_ID(quad_pos_msg));
	int quad_mode_sub = orb_subscribe(ORB_ID(quad_mode));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	orb_advert_t quad_velocity_sp_pub = orb_advertise(ORB_ID(quad_velocity_sp), &velocity_sp);
	orb_advert_t quad_mode_pub = orb_advertise(ORB_ID(quad_mode), &quad_mode);

	float	Kp_thrust = 0.1,//0.00008,
	        Kd_thrust = 0.11, /* Controller constants for thrust controller */
		Ki_thrust = 0.002,
	        Kp_pos = 0.3,
	        Kd_pos = 0.001, /* Controller constants for position controller */
                Ki_pos = 0.003,//0.001,
	 	
	 	hover_alt = 0.8,		// 1 meter altitude setpoint
	 	landing_alt = 0.3,
		hover_threashold = 0.2,
		anti_gravity = 0.46,
		min_rotor_speed = 0.25,
		pos_max = 0.3,
		int_max= 0.2,
		speed_up_time = 4,
		min_hover_velocity = 0.1,
		thrust_filter = 0.03,
		dt_pos = 0,
		time = 0,
		// print_timer = 0,
	        time_old = hrt_absolute_time() / (float)1000000;

	param_t param_ptr = param_find("MAV_SYS_ID");
	int32_t system_id;

	param_get(param_ptr, &system_id);

	int error_count = 0;

	bool    initialised = false,
                shutdown_motors = true,
                quad_mode_updated = false,
                vehicle_status_updated = false,
                system_error = false,
                test = false;

        velocity_t q_vel_ref;
        memset(&q_vel_ref, 0, sizeof(q_vel_ref));

	struct pollfd fds[1];
	fds[0].fd = quad_pos_sub;
	fds[0].events = POLLIN;

	while ( !thread_should_exit ) {

		int pret = poll(fds, 1, 300);
		if (pret < 0) {
			mavlink_log_info(mavlink_fd,"[POT%d] Poll error", system_id);
		} else if (pret == 0){
			if (vehicle_status.arming_state == ARMING_STATE_ARMED && !system_error){
				if (error_count >= 5){
					system_error = true;
					initialised = false;
					error_count = 0;

					shutdown(&velocity_sp, &quad_velocity_sp_pub);
					emergency(&quad_mode, &quad_mode_pub);

					quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
					orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);

					mavlink_log_critical(mavlink_fd,"[POT%d] Package loss limit reached", system_id);
				}
				error_count++;	
			}

		} else if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(quad_pos_msg), quad_pos_sub, &quad_pos);

			orb_check(quad_mode_sub, &quad_mode_updated);
			if (quad_mode_updated){
				orb_copy(ORB_ID(quad_mode), quad_mode_sub, &quad_mode);
			}

			orb_check(vehicle_status_sub, &vehicle_status_updated);
			if (vehicle_status_updated){
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);
			}

			if (!initialised && vehicle_status.arming_state == ARMING_STATE_ARMED){
				time = hrt_absolute_time() / (float)1000000;
				dt_pos = time - time_old;
				time_old = time;
				error_count = 0;

				orb_copy(ORB_ID(quad_mode), quad_mode_sub, &quad_mode);
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vehicle_status);

				memset(&output, 0, sizeof(output));
				memset(&state_transition, false, sizeof(state_transition));
				memset(&sp, 0, sizeof(sp));
				memset(&error, 0, sizeof(error));
				memset(&velocity_sp, 0, sizeof(velocity_sp));

				quad_mode.cmd = QUAD_CMD_PENDING;
				quad_mode.current_state = QUAD_STATE_GROUNDED;

				orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
				orb_publish(ORB_ID(quad_velocity_sp), quad_velocity_sp_pub, &velocity_sp);

				initialised = true;
				mavlink_log_info(mavlink_fd,"[POS%d] INITIALISED",system_id);

			} else {
				time = hrt_absolute_time() / (float)1000000;
	                        dt_pos = time - time_old;
	                        time_old = time;

	                        // mavlink_log_info(mavlink_fd,"dt_pos: %.3f",(double)dt_pos);	
			}


			// Set state values
			state.x_old = state.x;
			state.y_old = state.y;
			state.z_old = state.z;

			state.x = quad_pos.x[system_id - 1] / (float)1000;
			state.y = quad_pos.y[system_id - 1] / (float)1000;
			state.z = quad_pos.z[system_id - 1] / (float)1000;

			state.dx = (state.x - state.x_old) / (float)dt_pos;
			state.dy = (state.y - state.y_old) / (float)dt_pos;
			state.dz = (state.z - state.z_old) / (float)dt_pos;

			// Set initial values when received commands
			if ( vehicle_status.arming_state == ARMING_STATE_ARMED && !system_error ){

				// print_timer = print_timer + dt_pos;

				// if(print_timer > 5){
				// 	mavlink_log_info(mavlink_fd,"error_count: %d",(int)error_count);
				// 	print_timer = 0;
				// }

				if ( quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_TAKEOFF) {

					if ( !state_transition.takeoff ) {
						sp.timestamp = time;
						sp.x = state.x;
						sp.y = state.y;
						sp.z = hover_alt;

						state_transition.takeoff = true;
						shutdown_motors = false;
						mavlink_log_info(mavlink_fd,"[POT%d] TAKEOFF INITIALISED",system_id);
					
					} else {
						// Takeoff sequence
						if ((state.z > (sp.z - (float)hover_threashold)) && (state.z < (sp.z + (float)hover_threashold)) && ((float)fabs(state.dz) < min_hover_velocity)) {
							
							// Change state to hovering state
							state_transition.takeoff = false;
							quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
							quad_mode.current_state = (enum QUAD_STATE)QUAD_STATE_HOVERING;

							orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
							mavlink_log_info(mavlink_fd,"[POT%d] HOVERING", system_id);
						}
					} 

				} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_LAND) {
					
					if (!state_transition.land) {

                                                // initialise landing sequence
                                                sp.z = landing_alt;

                                                state_transition.land = true;
                                                mavlink_log_info(mavlink_fd,"[POT%d] LANDING INITIALISED",system_id);
					
					} else {

						if ((state.z > (sp.z - (float)hover_threashold)) && (state.z < (sp.z + (float)hover_threashold)) && ((float)fabs(state.dz) < min_hover_velocity)) {
							
							// Change state to hovering state

							sp.z = 0;
							shutdown_motors = true;
							state_transition.land = false;

							quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
							quad_mode.current_state = (enum QUAD_STATE)QUAD_STATE_GROUNDED;

							orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
							mavlink_log_info(mavlink_fd,"[POT%d] LANDED",system_id);
						}
					}

				} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_START_SWARM) {
        				if(!state_transition.start_swarm) {
        					Ki_pos = Ki_pos * (float)0.1;
        					state_transition.start_swarm = true;
                                        	quad_mode.current_state = QUAD_STATE_SWARMING;
                                        	orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
        					mavlink_log_info(mavlink_fd,"[POT%d] INITIALISED SWARMING",system_id);
        				}

                                        q_vel_ref = wall( state.x, state.y );

                                        sp.dx = (float)-1 * q_vel_ref.v1;
                                        sp.dy = q_vel_ref.v2;
					//}

                                        if ( state.y < (float)0.8 ) {
                                                sp.dy += (float)0.05;
                                        } /* else {
                                           *         /\* sp.dy = (float)0; *\/
                                           *         state_transition.start_swarm = false;
                                           *         quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
                                           *         orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
                                           * } */

                                        // mavlink_log_info(mavlink_fd,"[POT%d] non sp.dx = %.3f, sp.dy = %.3f",system_id, (double)sp.dx, (double)sp.dy);
                                        if ( (float)(sp.dx) > (float)0.1 )
                                                sp.dx = (float)0.1;

                                        if ( (float)(sp.dx) < (float)-0.1 )
                                                sp.dx = (float)-0.1;

                                        if ( (float)(sp.dy) > (float)0.1 )
                                                sp.dy = (float)0.1;

                                        if ( (float)(sp.dy) < (float)-0.1 )
                                                sp.dy = (float)-0.1;


                                        // mavlink_log_info(mavlink_fd,"[POT%d] lim sp.dx = %.3f, sp.dy = %.3f", system_id, (double)sp.dx, (double)sp.dy);
                                        sp.y = sp.y + (sp.dy * (float)dt_pos);
                                        sp.x = sp.x + (sp.dx * (float)dt_pos);
                                        
                                } else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_STOP_SWARM) {

                                        if ( !state_transition.stop_swarm ) {
                                                Ki_pos = Ki_pos * (float)10;
                                                sp.dx = (float)0;
                                                sp.dy = (float)0;
                                                state_transition.stop_swarm = true;
                                                mavlink_log_info(mavlink_fd,"[POT%d] STOPPING SWARM INITIALISED",system_id);
                                        
                                        } else {
                                                // STopping sequence
                                                if (((float)fabs(state.dx) < min_hover_velocity) && ((float)fabs(state.dy) < min_hover_velocity) ) {
                                                        
                                                        // Change state to hovering state
                                                        state_transition.stop_swarm = false;
                                                        quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
                                                        quad_mode.current_state = (enum QUAD_STATE)QUAD_STATE_HOVERING;

                                                        orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
                                                        mavlink_log_info(mavlink_fd,"[POT%d] HOVERING",system_id);
                                                }
                                        }

				} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_PENDING) {
					// mavlink_log_info(mavlink_fd,"[POT] PENDING");
				}

				// Thrust controller
				error.thrust = sp.z - state.z;
				error.thrust_der = (error.thrust - error.thrust_old) / (float)dt_pos;
				error.thrust_int = error.thrust_int + error.thrust;

				error.thrust_old = error.thrust;
				state.thrust_old = output.thrust;

				output.thrust = Kp_thrust * error.thrust + Kd_thrust * error.thrust_der + Ki_thrust * error.thrust_int;

				// Thrust filter
	                        if (output.thrust > state.thrust_old + thrust_filter){
	                                output.thrust = state.thrust_old + thrust_filter;
	                        } else if (output.thrust < state.thrust_old - thrust_filter) {
	                                output.thrust = state.thrust_old - thrust_filter;
	                        }


	                        // Position controller
	                        error.x = sp.x - state.x;
	                        error.y = sp.y - state.y;

	                        error.dx = sp.dx - (error.x - error.x_old) / (float)dt_pos;
	                        error.dy = sp.dy - (error.y - error.y_old) / (float)dt_pos;

	                        error.x_old = error.x;
	                        error.y_old = error.y;

	                        error.x_int = error.x_int + error.x;
	                        error.y_int = error.y_int + error.y;

	                        if ( ((float)fabs(error.x_int) * (float)Ki_pos) > (float)int_max )
	                                error.x_int = int_max * error.x_int / ((float)fabs(error.x_int) * Ki_pos);

	                        if ( ((float)fabs(error.y_int) * (float)Ki_pos) > (float)int_max )
	                                error.y_int = int_max * error.y_int / ((float)fabs(error.y_int) * Ki_pos);


	                        output.pitch = - Kp_pos * error.x - Kd_pos * error.dx - Ki_pos * error.x_int;
	                        output.roll  = - Kp_pos * error.y - Kd_pos * error.dy - Ki_pos * error.y_int;


	                        /* Limiting position controller output */
	                        if ((float)fabs(output.roll) > pos_max)
	                                output.roll = pos_max * (output.roll / (float)fabs(output.roll));

	                        if ((float)fabs(output.pitch) > pos_max)
	                                output.pitch = pos_max * output.pitch / (float)fabs(output.pitch);

	                        velocity_sp.roll = output.roll;
	                        velocity_sp.pitch = output.pitch;

	                        if ((sp.timestamp + (float)speed_up_time) > time){
					velocity_sp.thrust = min_rotor_speed;
					error.thrust_int = 0;
					error.x_int = 0;
					error.y_int = 0;

					sp.x = state.x;
					sp.y = state.y;
				} else {
					// Set thrust output
					velocity_sp.thrust = output.thrust + anti_gravity;

				}

	                        // Thrust limiter
	                        if ( velocity_sp.thrust > (float)1 ) {
	                                velocity_sp.thrust = (float)1;

	                        } else if ( velocity_sp.thrust < 0 ) {
	                                velocity_sp.thrust = 0;
	                        }
	                        
	                        if(test || shutdown_motors) {
	                        	velocity_sp.thrust = 0;
	                        	velocity_sp.roll = 0;
	                        	velocity_sp.pitch = 0;
	         	                velocity_sp.yaw = 0;
	                        }

	                        // Publish the new roll, pitch, yaw and thrust set points
	                        // mavlink_log_info(mavlink_fd,"th: %.3f", (double)velocity_sp.thrust);
	                        orb_publish(ORB_ID(quad_velocity_sp), quad_velocity_sp_pub, &velocity_sp);

			} else if (vehicle_status.arming_state == ARMING_STATE_STANDBY) {

				initialised = false;
				system_error = false;
				error_count = 0;

				if (velocity_sp.thrust != 0) {
					velocity_sp.thrust = 0;
		                        velocity_sp.roll = 0;
		                        velocity_sp.pitch = 0;
		         	        velocity_sp.yaw = 0;

		         	        orb_publish(ORB_ID(quad_velocity_sp), quad_velocity_sp_pub, &velocity_sp);
				}

				if ( quad_mode.current_state != (enum QUAD_STATE)QUAD_STATE_GROUNDED || quad_mode.cmd != (enum QUAD_CMD)QUAD_CMD_PENDING ) {
					quad_mode.cmd = (enum QUAD_CMD)QUAD_CMD_PENDING;
					quad_mode.current_state = (enum QUAD_STATE)QUAD_STATE_GROUNDED;

					orb_publish(ORB_ID(quad_mode), quad_mode_pub, &quad_mode);
				}
			}
		}
	}
}

void emergency(struct quad_mode_s *mode, orb_advert_t *mode_pub) {
	mode->error = true;
	orb_publish(ORB_ID(quad_mode), *mode_pub, mode);

	mode->current_state = QUAD_STATE_GROUNDED;

	orb_publish(ORB_ID(quad_mode), *mode_pub, mode);
}

void shutdown( struct quad_velocity_sp_s *sp, orb_advert_t *velocity_sp_pub ){
	sp->thrust = 0;
	sp->roll = 0;
	sp->pitch = 0;
	sp->yaw = 0;

	orb_publish(ORB_ID(quad_velocity_sp), *velocity_sp_pub, sp);
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
