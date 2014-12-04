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
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/quad_att_sp.h>
#include <uORB/topics/quad_formation_msg.h>
#include <uORB/topics/vehicle_attitude.h>

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

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

int quad_commander_thread_main(int argc, char *argv[]) {
        warnx("[quad_commander] has begun");

        static int mavlink_fd;
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        mavlink_log_info(mavlink_fd, "[quad_att__control] started");

        /* Subscription */
        struct quad_swarm_cmd_s swarm_cmd;
        memset(&swarm_cmd, 0, sizeof(swarm_cmd));

        int cmd_sub = orb_subscribe(ORB_ID(quad_swarm_cmd));

        /* Published */
        struct quad_mode_s mode;
        memset(&mode, 0, sizeof(mode));

        orb_advert_t mode_pub = orb_advertise(ORB_ID(quad_mode), &mode);

        struct pollfd fd_cmd[1]; /* polles på cmd mavlink */
        fd_cmd[0].fd = cmd_sub;
        fd_cmd[0].events = POLLIN;

        while (!thread_should_exit) {
                int ret_cmd = poll(fd_cmd, 1, 100);
                if (ret_cmd < 0) {
                        warnx("poll cmd error");
                } else if (ret_cmd == 0) {
                        /* no return value - nothing has happened */
                } else if (fd_cmd[0].revents & POLLIN) {
                        orb_copy(ORB_ID(quad_quad_swarm_cmd), cmd_sub, &cmd);
                        
                        if ( cmd.cmd == (uint8_t)QUAD_CMD_TAKEOFF ) {
                                
                        } else if ( cmd == (uint8_t)QUAD_CMD_SWARM ) {

                        } else if ( cmd == (uint8_t)QUAD_CMD_LAND ) {

                        } else if ( cmd == (uint8_)QUAD_CMD_GROUNDED ) {
                        
                        } else {
                                /* nothing to do */
                        }

                } else {
                        /* nothing happened */
                }
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
