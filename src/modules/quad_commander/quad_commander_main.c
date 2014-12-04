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

#include "quad_commander_main.h"


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
        struct quad_formation_msg_s qmsg; /* Her skal det være den nye cmd msg */
        memset(&qmsg, 0, sizeof(qmsg));

        int qmsg_sub = orb_subscribe(ORB_ID(quad_formation_msg));

        /* Published */
        struct actuator_controls_s actuators; /* Her skal publishes på cmd topic */
        memset(&actuators, 0, sizeof(actuators));

        orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

        struct pollfd fd_v_att[1]; /* polles på cmd mavlink */
        fd_v_att[0].fd = v_att_sub;
        fd_v_att[0].events = POLLIN;

        while (!thread_should_exit) {
                int ret_sp = poll(fd_sp, 1, 100);
                if (ret_sp < 0) {
                        warnx("poll sp error");
                } else if (ret_sp == 0) {
                        /* no return value - nothing has happened */
                } else if (fd_sp[0].revents & POLLIN) {
                        orb_copy(ORB_ID(quad_att_sp), quad_sp_sub, &sp);

                        
                        if ( cmd == state transition 1 ) {
                                /* sæt parameter så statesne passer */
                        } else if ( cmd == state transition 2 ) {

                        } else if ( cmd == state transition 3 ) {

                        } else if ( cmd == state transition 4 ) {
                        
                        } else if ( cmd == state transition 5 ) {
                        
                        } else if ( cmd == state transition 6 ) {
                        
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
