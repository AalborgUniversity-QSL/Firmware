/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/
/*
 * @file main.c
 * 
 * Implementation of an quadrotor formation control app for use in the 
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

#include <v1.0/quad_formation/mavlink.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_command.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* Function prototypes */
__EXPORT int q_formation_control_main(int argc, char *argv[]);
int formation_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

int formation_control_thread_main(int argc, char *argv[]) {

        warnx("[Formation_control] started");

        static int mavlink_fd;
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        mavlink_log_info(mavlink_fd, "[q_formation_control @ mavlink] Halløj Jens, program lige startet.");
        printf("[q_formation_control @ serial] Halløj Jens, program lige startet.\n");
        
        int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

        orb_set_interval(sensor_sub_fd, 250);

	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
        orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

        int vcmd_sub = orb_subscribe(ORB_ID(vehicle_command)); /* handle til vehicle command */
        struct vehicle_command_s vcmd;
        orb_copy(ORB_ID(vehicle_command), vcmd_sub, &vcmd);

        struct pollfd fds[] = { 
            { .fd = sensor_sub_fd,   .events = POLLIN },
        };

        struct pollfd fd_cmd[] = { 
                { .fd = vcmd_sub,   .events = POLLIN },
        };
        
        struct sensor_combined_s raw;
        orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
        float gnd_alt = raw.baro_alt_meter;
        
        bool y_first = false;
        bool i_first = false;

        while(!thread_should_exit) {
                if (y_first == false) {
                        mavlink_log_info(mavlink_fd, "[q_formation_control @ mavlink] Halløj Jens, program i yderste loop."); /* sender strend via mavlink */
                        printf("[q_formation_control @ serial] Halløj Jens, program i yderste loop.\n"); /* sender streng via seriel terminal */
                        y_first = true;
                }
                int ret_cmd = poll(fd_cmd, 1, 250);
                if (ret_cmd < 0) {
			warnx("poll cmd error");
		} else if (ret_cmd == 0) {
			/* no return value - nothing has happened */
		} else {
                        if (fd_cmd[0].revents & POLLIN) {
                                orb_copy(ORB_ID(vehicle_command), vcmd_sub, &vcmd);
                                if (vcmd.command == MAV_VEHICLE_CMD_FORMATION_START) {
                                        while (!thread_should_exit) {
                                                if (i_first == false) {
                                                        mavlink_log_info(mavlink_fd, "[q_formation_control @ mavlink] Halløj Jens, program i inderste loop."); /* sender streng via mavlink */
                                                        printf("[q_formation_control @ serial] Halløj Jens, program i inderste loop.\n"); /* sender streng via seriel terminal */
                                                        i_first = true;
                                                }
                                                int ret = poll(fds, 1, 250);
                                                if (ret < 0) {
                                                        warnx("poll error");
                                                } else if (ret == 0) {
                                                        /* no return value - nothing has happened */
                                                } else {
                                                        if (fds[0].revents & POLLIN) { /* if there is new data */
                                                                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

                                                                if ( raw.baro_alt_meter < (gnd_alt + 10) ) { /* Skal det her være i meter eller? */
                                                                        att_sp.thrust += (float)0.2;
                                                                        att_sp.q_d[0] = 0;
                                                                        att_sp.q_d[1] = 0;
                                                                        att_sp.q_d[2] = 0;
                                                                        att_sp.q_d[3] = 1;
                                                                        att_sp.q_d_valid = true;
                                                                        orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
                                                                } else if ( raw.baro_alt_meter >= (gnd_alt + 10) ) { /* Skal det her være i meter eller? */
                                                                        att_sp.thrust -= (float)0.2;
                                                                        att_sp.q_d[0] = 0;
                                                                        att_sp.q_d[1] = 0;
                                                                        att_sp.q_d[2] = 0;
                                                                        att_sp.q_d[3] = 1;
                                                                        att_sp.q_d_valid = true;
                                                                        orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
                                                                } else {
                                                                        /* this is bad */
                                                                }
                                                        }
                                                }
                                        }
                                } else {
                                        /* Ingen kommando ankommet */
                                }
                        }
                }
        }
}

static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: q_formation_control {start|stop|status}\n\n");
        exit(1);
}

int q_formation_control_main(int argc, char *argv[]) {
        if (argc < 1)
		usage("missing argument");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("formation_control already running\n");

			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("formation_control",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 20,
                                             2048,
                                             formation_control_thread_main,
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
			printf("\tformation_control is running\n");

		} else {
			printf("\tformation_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
