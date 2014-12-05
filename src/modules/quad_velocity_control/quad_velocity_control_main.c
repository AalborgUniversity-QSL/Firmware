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

#include <systemlib/param/param.h>

#include <mavlink/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/quad_velocity_sp.h>
#include <uORB/topics/quad_pos_msg.h>
#include <uORB/topics/quad_mode.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <param.h>

__EXPORT int quad_velocity_control_main(int argc, char *argv[]);
int quad_velocity_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);


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
	struct state_transition_s state_transition;
	memset(&state_transition, false, sizeof(state_transition));
	struct quad_alt_velocity_sp alt_velocity_sp;
	memset(&alt_velocity_sp, 0, sizeof(alt_velocity_sp));

	int quad_pos_sub = orb_subscribe(ORB_ID(quad_pos_msg));
	int quad_mode_sub = orb_subscribe(ORB_ID(quad_mode));

	orb_advert_t quad_velocity_sp_pub = orb_advertise(ORB_ID(quad_velocity_sp), &velocity_sp);
	orb_advert_t quad_mode_sub = orb_advertise(ORB_ID(quad_mode), &quad_mode);

	int package_loss = 0;

	int system_id = 1;

	struct pollfd fds[1];
	fds[0].fd = quad_pos_sub;
	fds[0].events = POLLIN;

	while(!thread_should_exit){

		int pret = poll(fds, 1, 500);

		if (pret < 0){
			mavlink_log_info(mavlink_fd,"[POT] Poll error");
		} else if (pret == 0){
			package_loss++;
			if(package_loss > 10){
				package_loss = 0;
				mavlink_log_info(mavlink_fd,"[POT] Package loss limit reached")
			}
		} else if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(quad_pos_msg), quad_pos_sub, &quad_pos);

			bool quad_mode_updated;
			orb_check(quad_mode_sub, &quad_mode_updated);

			if (quad_mode_updated){
				orb_copy(ORB_ID(quad_mode), quad_mode_sub, quad_mode);
			}

			if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_TAKEOFF && !state_transition.takeoff){
				
				// initialise take-off sequence
				struct init_pos_s takeoff_pos;
				memset(&takeoff_pos, 0, sizeof(takeoff_pos));

				takeoff_pos.timestamp = (hrt_absolute_time() / (float)1000000);
				takeoff_pos.x = quad_pos.x[system_id - 1];
				takeoff_pos.y = quad_pos.y[system_id - 1];
				takeoff_pos.z = quad_pos.z[system_id - 1];

				alt_velocity_sp.timestamp = (hrt_absolute_time() / (float)1000000);
				alt_velocity_sp.dx = 0;
				alt_velocity_sp.dy = 0;
				alt_velocity_sp.altitude = formation_alt;

				state_transition.takeoff = true;

			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_LAND && !state_transition.land){
				// Landing sequence

			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_START_SWARM && !state_transition.start){
				// Start computing potential fields
			
			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_STOP_SWARM && !state_transition.stop){
				
				// Stop computing potential fields and break formation and return to hovering
				struct init_pos_s landing_pos;
				memset(&landing_pos, 0, sizeof(landing_pos));

				landing_pos.timestamp = (hrt_absolute_time() / (float)1000000);
				landing_pos.x = quad_pos.x[system_id - 1];
				landing_pos.y = quad_pos.y[system_id - 1];
				landing_pos.z = quad_pos.z[system_id - 1];

				state_transition = true;

			} else {
				// Do nothing yet

			}



			if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_TAKEOFF && state_transition.takeoff){
				// Takeoff sequence
				if (quad_pos.z[system_id - 1] > 100 && quad_mode.current_state == (enum QUAD_STATE)QUAD_STATE_GROUNDED){
					state_transition.in_air = true;
				}

				if (quad_pos.z[system_id - 1] < (formation_alt + (float)altitude_threashold && quad_pos.z[system_id - 1] > (formation_alt - (float)altitude_threashold ){
					quad_mode.current_state = (enum QUAD_STATE)QUAD_STATE_HOVERING;
				}

			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_LAND && state_transition.land){

			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_START_SWARM && state_transition.start){

			} else if (quad_mode.cmd == (enum QUAD_CMD)QUAD_CMD_STOP_SWARM && state_transition.stop)

		}

	}
	return 0;
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
                                             SCHED_PRIORITY_MAX - 6,
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
