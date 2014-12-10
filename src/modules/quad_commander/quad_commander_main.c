/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************
 *
 * @file main.c
 *
 * Implementation of an quadrotor commander app for use in the
 * semester project.
 *
 */

#include <nuttx/config.h>
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

#include <mavlink/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/quad_mode.h>
#include <uORB/topics/quad_swarm_cmd.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/quad_pos_msg.h>

#include <geo/geo.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <lib/mathlib/mathlib.h>

#define BUG(x) mavlink_log_info(mavlink_fd, "[quad_commander] Debug no. %i", x); /* to ease debug messages */

/**
 * Main loop starter
 */
__EXPORT int quad_commander_main(int argc, char *argv[]);

/**
 * Main loop
 */
int quad_commander_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Error messages via mavlink
 */
void error_msg( int error, bool *transition_error );

/**
 * State transitions functions
 */
int take_off( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub );
int land( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub, bool *transition_error );
int emergency_land( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub );
int start_swarm( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub );
int stop_swarm( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub );

/**
 * Globals
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
static bool low_battery = false;

static int mavlink_fd;
const int time_out = 20000;    /* Timeout value for state transition poll [ms] */


int quad_commander_thread_main(int argc, char *argv[]) {
        warnx("[quad_commander] has begun");

        /**
         * Subscriptions
         */
        struct quad_swarm_cmd_s swarm_cmd;
        struct quad_mode_s state;
        struct vehicle_status_s v_status;

        memset(&swarm_cmd, 0, sizeof(swarm_cmd));
        memset(&state, 0, sizeof(state));
        memset(&v_status, 0, sizeof(v_status));

        int swarm_cmd_sub = 0;
        int state_sub = 0;
        int v_status_sub = 0;

        swarm_cmd_sub = orb_subscribe(ORB_ID(quad_swarm_cmd));
        state_sub = orb_subscribe(ORB_ID(quad_mode));
        v_status_sub = orb_subscribe(ORB_ID(vehicle_status));

        /**
         * Topics to be published on
         */
        struct quad_mode_s mode;
        orb_advert_t mode_pub = orb_advertise(ORB_ID(quad_mode), &mode);
        memset(&mode, 0, sizeof(mode));

        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        mavlink_log_info(mavlink_fd, "[quad_commander] started");

        struct pollfd fd_cmd[1];
        fd_cmd[0].fd = swarm_cmd_sub;
        fd_cmd[0].events = POLLIN;

        struct pollfd fd_state;
        fd_state.fd = state_sub;
        fd_state.events = POLLIN;

        /* Initial state of the quadrotor; operations always start from the ground */
        state.current_state = QUAD_STATE_GROUNDED;
        mode.current_state = QUAD_STATE_GROUNDED;
        orb_publish(ORB_ID(quad_mode), mode_pub, &mode);

        bool transition_error  = false;

        while ( !thread_should_exit ) {
                bool v_status_updated;
                orb_check(v_status_sub, &v_status_updated);
                if ( v_status_updated )
                        orb_copy(ORB_ID(vehicle_status), v_status_sub, &v_status);

                if ( v_status.arming_state == ARMING_STATE_STANDBY )
                        state.current_state == QUAD_STATE_GROUNDED;

                if ( (v_status.battery_warning == VEHICLE_BATTERY_WARNING_LOW) &&
                     (state.current_state != QUAD_STATE_GROUNDED) ) {

                        low_battery = true;
                        mavlink_log_critical(mavlink_fd, "[quad_commmander] Battery lavel low!");
                        emergency_land( &state, &mode, &mode_pub, &state_sub );

                } else {
                        low_battery = false;
                }

                int ret_state = poll(&fd_state, 1, 1);
                if ( ret_state < 0 ) {
                        warnx("poll cmd error");
                } else if ( ret_state == 0 ) {
                        /* nothing happened */
                } else if ( fd_state.revents & POLLIN ) {
                        orb_copy(ORB_ID(quad_mode), state_sub, &state);
                } else {
                        /* nothing happened */
                }

                if ( state.error == true ) {
                        int ret_value = emergency_land( &state, &mode, &mode_pub, &state_sub );
                        error_msg( ret_value, &transition_error );
                }

                int ret_cmd = poll(fd_cmd, 1, 250);
                if (ret_cmd < 0) {
                        warnx("poll cmd error");
                } else if (ret_cmd == 0) {
                        /* no return value - nothing has happened */
                } else if (fd_cmd[0].revents & POLLIN) {
                        orb_copy(ORB_ID(quad_swarm_cmd), swarm_cmd_sub, &swarm_cmd);

                        if ( swarm_cmd.cmd_id == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_TAKEOFF ) {
                                mavlink_log_info(mavlink_fd, "[quad_commmander] Takeoff transition begun!");
                                int ret_value = take_off( &state, &mode, &mode_pub, &state_sub );
                                error_msg( ret_value, &transition_error );

                        } else if ( swarm_cmd.cmd_id == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_LAND ) {
                                mavlink_log_info(mavlink_fd, "[quad_commmander] Landing transition begun!");
                                int ret_value = land( &state, &mode, &mode_pub, &state_sub, &transition_error );
                                error_msg( ret_value, &transition_error );

                        } else if ( swarm_cmd.cmd_id == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_START_SWARM ) {
                                mavlink_log_info(mavlink_fd, "[quad_commmander] Swarming transition begun!");
                                int ret_value = start_swarm( &state, &mode, &mode_pub, &state_sub );
                                error_msg( ret_value, &transition_error );

                        } else if ( swarm_cmd.cmd_id == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_STOP_SWARM ) {
                                mavlink_log_info(mavlink_fd, "[quad_commmander] Stop swarming transition begun!");
                                int ret_value = stop_swarm( &state, &mode, &mode_pub, &state_sub );
                                error_msg( ret_value, &transition_error );

                        } else {
                                /* Nothing to do */
                        }

                } else {
                        /* nothing happened */
                }
        }
}

void error_msg( int error, bool *transition_error ) {
        if ( error == 0 ){
                mavlink_log_info(mavlink_fd, "[quad_commmander] State transition succes!");

        } else if ( error == -1 ) {
                mavlink_log_info(mavlink_fd, "[quad_commmander] No state transition occured!");

        } else if ( error == -2 ) {
                mavlink_log_info(mavlink_fd, "[quad_commmander] Not in correct state!");

        } else if ( error == -3 ) {
                mavlink_log_info(mavlink_fd, "[quad_commmander] Not changed to the correct state!");

        } else {
                mavlink_log_info(mavlink_fd, "[quad_commmander] Unknown return value!");

        }

        if ( error == -1 )
                *transition_error = true;

}

int take_off( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub ) {
        if ( (state->current_state == (enum QUAD_STATE)QUAD_STATE_GROUNDED) ) {
                mode->cmd = (enum QUAD_CMD)QUAD_CMD_TAKEOFF;
                orb_publish(ORB_ID(quad_mode), *mode_pub, mode);
                orb_copy(ORB_ID(quad_mode), *state_sub, state);

                struct pollfd fd_state;
                fd_state.fd = *state_sub;
                fd_state.events = POLLIN;

                int ret_state = poll(&fd_state, 1, time_out);
                if ( ret_state < 0 ) {
                        warnx("poll cmd error");
                } else if ( ret_state == 0 ) {
                        return -1;

                } else if ( fd_state.revents & POLLIN ) {
                        orb_copy(ORB_ID(quad_mode), *state_sub, state);

                        if ( state->current_state == (enum QUAD_STATE)QUAD_STATE_HOVERING )
                                return 0;

                        return -3;

                } else {
                        mavlink_log_info(mavlink_fd, "[quad_commmander] something is very wrong");

                }

        } else {
                return -2;
        }
}

int land( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub, bool *transition_error ) {
        if ( (state->current_state == (enum QUAD_STATE)QUAD_STATE_HOVERING) || *transition_error ) {
                mode->cmd = (enum QUAD_CMD)QUAD_CMD_LAND;
                orb_publish(ORB_ID(quad_mode), *mode_pub, mode);
                orb_copy(ORB_ID(quad_mode), *state_sub, state);

                struct pollfd fd_state;
                fd_state.fd = *state_sub;
                fd_state.events = POLLIN;

                int ret_state = poll(&fd_state, 1, time_out);
                if ( ret_state < 0 ) {
                        warnx("poll cmd error");
                } else if ( ret_state == 0 ) {
                        return -1;

                } else if ( fd_state.revents & POLLIN ) {
                        orb_copy(ORB_ID(quad_mode), *state_sub, state);
                        if ( state->current_state == (enum QUAD_STATE)QUAD_STATE_GROUNDED ) {
                                return 0;
                                *transition_error = false;
                        }
                        return -3;

                } else {
                        mavlink_log_info(mavlink_fd, "[quad_commmander] something is very wrong");

                }

        } else {
                return -2;
        }
}

int emergency_land( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub ) {
        mode->cmd = (enum QUAD_CMD)QUAD_CMD_LAND;
        orb_publish(ORB_ID(quad_mode), *mode_pub, mode);
        orb_copy(ORB_ID(quad_mode), *state_sub, state);

        struct pollfd fd_state;
        fd_state.fd = *state_sub;
        fd_state.events = POLLIN;

        int ret_state = poll(&fd_state, 1, time_out);
        if ( ret_state < 0 ) {
                warnx("poll cmd error");
        } else if ( ret_state == 0 ) {
                return -1;

        } else if ( fd_state.revents & POLLIN ) {
                orb_copy(ORB_ID(quad_mode), *state_sub, state);
                if ( state->current_state == (enum QUAD_STATE)QUAD_STATE_GROUNDED )
                        return 0;

                return -3;

        } else {
                mavlink_log_info(mavlink_fd, "[quad_commmander] something is very wrong");

        }
        state->error = false;
}

int start_swarm( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub ) {
        if ( state->current_state == (enum QUAD_STATE)QUAD_STATE_HOVERING /* && !low_battery */ ) {
                mode->cmd = (enum QUAD_CMD)QUAD_CMD_START_SWARM;
                orb_publish(ORB_ID(quad_mode), *mode_pub, mode);
                orb_copy(ORB_ID(quad_mode), *state_sub, state);

                struct pollfd fd_state;
                fd_state.fd = *state_sub;
                fd_state.events = POLLIN;

                int ret_state = poll(&fd_state, 1, time_out);
                if ( ret_state < 0 ) {
                        warnx("poll cmd error");
                } else if ( ret_state == 0 ) {
                        return -1;

                } else if ( fd_state.revents & POLLIN ) {
                        orb_copy(ORB_ID(quad_mode), *state_sub, state);
                        if ( state->current_state == (enum QUAD_STATE)QUAD_STATE_SWARMING )
                                return 0;

                        return -3;

                } else {
                        mavlink_log_info(mavlink_fd, "[quad_commmander] something is very wrong");

                }

        } else {
                return -2;
        }
}

int stop_swarm( struct quad_mode_s *state, struct quad_mode_s *mode, orb_advert_t *mode_pub, int *state_sub ) {
        if ( state->current_state == (enum QUAD_STATE)QUAD_STATE_SWARMING ) {
                mode->cmd = (enum QUAD_CMD)QUAD_CMD_STOP_SWARM;
                orb_publish(ORB_ID(quad_mode), *mode_pub, mode);
                orb_copy(ORB_ID(quad_mode), *state_sub, state);

                struct pollfd fd_state;
                fd_state.fd = *state_sub;
                fd_state.events = POLLIN;

                int ret_state = poll(&fd_state, 1, time_out);
                if ( ret_state < 0 ) {
                        warnx("poll cmd error");
                } else if ( ret_state == 0 ) {
                        return -1;

                } else if ( fd_state.revents & POLLIN ) {
                        orb_copy(ORB_ID(quad_mode), *state_sub, state);
                        if ( state->current_state == (enum QUAD_STATE)QUAD_STATE_HOVERING )
                                return 0;

                        return -3;

                } else {
                        mavlink_log_info(mavlink_fd, "[quad_commmander] something is very wrong");

                }

        } else {
                return -2;
        }
}

static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: quad_commander {start|stop|status}\n\n");
        exit(1);
}

int quad_commander_main(int argc, char *argv[]) {
        if (argc < 1)
                usage("missing argument");

        if (!strcmp(argv[1], "start")) {

                if (thread_running) {
                        printf("quad_commander already running\n");

                        exit(0);
                }

                thread_should_exit = false;
                daemon_task = task_spawn_cmd("quad_commander",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 40,
                                             2048,
                                             quad_commander_thread_main,
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
                        printf("\tquad_commander is running\n");

                } else {
                        printf("\tquad_commander not started\n");
                }

                exit(0);
        }

        usage("unrecognized command");
        exit(1);
}
