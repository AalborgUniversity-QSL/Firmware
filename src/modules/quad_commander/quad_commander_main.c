/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/
/*
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
#include <uORB/topics/quad_swarm_cmd>
#include <uORB/topics/vehicle_status.h>

#include <geo/geo.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <lib/mathlib/mathlib.h>


/* Function prototypes */
__EXPORT int quad_commander_main(int argc, char *argv[]);
int quad_commander_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
int take_off(void);
int land(void);
int emergency_land(void);
int start_swarm(void);
int stop_swarm(void);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;
bool low_battery = false;

/* Subscription */
struct quad_swarm_cmd_s swarm_cmd;
memset(&swarm_cmd, 0, sizeof(swarm_cmd));
struct quad_pos_msg_s quad_pos;
memset(&quad_pos, 0, sizeof(quad_pos));
struct quad_mode_s mode_response;
memset(&mode_response, 0, sizeof(mode_response));
struct vehicle_status_s v_status;
memset(&v_status, 0, sizeof(v_status));

int swarm_cmd_sub = orb_subscribe(ORB_ID(quad_swarm_cmd));
int quad_pos_sub = orb_subscribe(ORB_ID(quad_pos_msg));
int mode_response_sub = orb_subscribe(ORB_ID(quad_mode), &mode_response);
int v_status_sub = orb_subscribe(ORB_ID(vehicle_status), &v_status);

/* Published */
struct quad_mode_s mode;
memset(&mode, 0, sizeof(mode));

orb_advert_t mode_pub = orb_advertise(ORB_ID(quad_mode), &mode);


int quad_commander_thread_main(int argc, char *argv[]) {
        warnx("[quad_commander] has begun");

        static int mavlink_fd;
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        mavlink_log_info(mavlink_fd, "[quad_att__control] started");

        struct pollfd fd_cmd[1]; /* polles på cmd mavlink */
        fd_cmd[0].fd = swarm_cmd_sub;
        fd_cmd[0].events = POLLIN;

        while (!thread_should_exit) {
                orb_copy(ORB_ID(vehicle_status), v_status_sub, &v_status);

                if ( v_status.battery_warning == (enum VEHICLE_BATTERY_WARNING)VEHICLE_BATTERY_WARNING_LOW ) {
                        low_battery = true;
                        mavlink_log_critical(mavlink_fd, "[quad_commmander] Battery lavel critical!");
                        emergency_land();

                } else {
                        low_battery = false;
                }

                int ret_cmd = poll(fd_cmd, 1, 100);
                if (ret_cmd < 0) {
                        warnx("poll cmd error");
                } else if (ret_cmd == 0) {
                        /* no return value - nothing has happened */
                } else if (fd_cmd[0].revents & POLLIN) {
                        orb_copy(ORB_ID(quad_quad_swarm_cmd), swarm_cmd_sub, &swarm_cmd);

                        if ( swarm_cmd.cmd == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_TAKEOFF ) {
                                take_off();

                        } else if ( swarm_cmd.cmd == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_LAND ) {
                                land();

                        } else if ( swarm_cmd.cmd == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_START_SWARM ) {
                                start_swarm();

                        } else if ( swarm_cmd.cmd == (enum QUAD_MSG_CMD)QUAD_MSG_CMD_STOP_SWARM ) {
                                stop_swarm();

                        } else {
                                /* Nothing to do */
                        }

                } else {
                        /* nothing happened */
                }
        }
}

int take_off(void) {
        if ( mode_response = (enum QUAD_STATE)QUAD_STATE_GROUNDED && !low_battery ) {
                mode.cmd == (enum QUAD_CMD)QUAD_CMD_TAKEOFF;
                orb_publish(ORB_ID(quad_mode), mode_pub, &mode);

                bool mode_response_updated;
                do {
                        orb_check(mode_response_sub, &mode_response_updated);
                        if ( mode_response_updated )
                                orb_copy(ORB_ID(quad_mode), mode_response_sub, &mode_response);

                } while ( mode_response.response != (enum QUAD_STATE)QUAD_STATE_HOVERING );
        } else {
                mavlink_log_critical(mavlink_fd, "[quad_commmander] Not in landed state!");
        }
}

int land(void) {
        if ( mode_response == (enum QUAD_STATE)QUAD_STATE_HOVERING ) {
                mode.cmd = (enum QUAD_CMD)QUAD_CMD_LAND;
                orb_publish(ORB_ID(quad_mode), mode_pub, &mode);

                bool mode_response_updated;
                do {
                        orb_check(mode_response_sub, &mode_response_updated);
                        if ( mode_response_updated )
                                orb_copy(ORB_ID(quad_mode), mode_response_sub, &mode_response);

                } while ( mode_response.response != (enum QUAD_STATE)QUAD_STATE_GROUNDED );

        } else {
                mavlink_log_critical(mavlink_fd, "[quad_commmander] Not in hover state!");
        }
}

int emergency_land(void) {
        mode.cmd = (enum QUAD_CMD)QUAD_CMD_LAND;
        orb_publish(ORB_ID(quad_mode), mode_pub, &mode);

        bool mode_response_updated;
        do {
                orb_check(mode_response_sub, &mode_response_updated);
                if ( mode_response_updated )
                        orb_copy(ORB_ID(quad_mode), mode_response_sub, &mode_response);

        } while ( mode_response.response != (enum QUAD_STATE)QUAD_STATE_GROUNDED );
}

int start_swarm(void) {
        if ( mode_response == (enum QUAD_STATE)QUAD_STATE_HOVERING && !low_battery ) {
                mode.cmd = (enum QUAD_CMD)QUAD_CMD_START_SWARM;
                orb_publish(ORB_ID(quad_mode), mode_pub, &mode);

                bool mode_response_updated;
                do {
                        orb_check(mode_response_sub, &mode_response_updated);
                        if ( mode_response_updated )
                                orb_copy(ORB_ID(quad_mode), mode_response_sub, &mode_response);

                } while ( mode_response.response != (enum QUAD_STATE)QUAD_STATE_SWARMING );

        } else {
                mavlink_log_critical(mavlink_fd, "[quad_commmander] Not in hover state!");
        }
}

int stop_swarm(void) {
        if ( mode_response == (enum QUAD_STATE)QUAD_STATE_SWARMING ) {
                mode.cmd = (enum QUAD_CMD)QUAD_CMD_STOP_SWARM;
                orb_publish(ORB_ID(quad_mode), mode_pub, &mode);

                bool mode_response_updated;
                do {
                        orb_check(mode_response_sub, &mode_response_updated);
                        if ( mode_response_updated )
                                orb_copy(ORB_ID(quad_mode), mode_response_sub, &mode_response);

                } while ( mode_response.response != (enum QUAD_STATE)QUAD_STATE_HOVERING );

        } else {
                mavlink_log_critical(mavlink_fd, "[quad_commmander] Not in swarming state!");
        }
}

static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: quad_commander {start|stop|status}\n\n");
        exit(1);
}

int quad_att_control_main(int argc, char *argv[]) {
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
                                             SCHED_PRIORITY_MAX - 5,
                                             2048,
                                             quad_commmander_thread_main,
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
