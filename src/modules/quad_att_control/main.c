/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/
/*
 * @file main.c
 * 
 * Implementation of an quadrotor attitude control app for use in the 
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
#include <uORB/topics/vehicle_attitude.h>

#include <geo/geo.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* Function prototypes */
__EXPORT int quad_att_control_main(int argc, char *argv[]);
int att_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

int att_control_thread_main(int argc, char *argv[]) {
        warnx("[quad_att_control] has begun");

        static int mavlink_fd;
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        mavlink_log_info(mavlink_fd, "[quad_att__control] started");
        
        /* Subscription */
        struct sensor_combined_s raw;
        memset(&raw, 0, sizeof(raw));
        struct quad_att_sp_s sp;
        memset(&sp, 0, sizeof(sp));
        struct vehicle_attitude_s v_att;
        memset(&v_att, 0, sizeof(v_att));

        int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
        int quad_sp_sub = orb_subscribe(ORB_ID(quad_att_sp));
        int v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

        /* Published */
        struct actuator_controls_s actuators;
        memset(&actuators, 0, sizeof(actuators));
        
        for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
                actuators.control[i] = 0.0f;
	}
        orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

        struct pollfd fds[] = { 
                { .fd = sensor_sub,   .events = POLLIN },
        };
        struct pollfd fd_sp[] = { 
                { .fd = quad_sp_sub,   .events = POLLIN },
        };
        struct pollfd fd_v_att[] = { 
                { .fd = v_att_sub,   .events = POLLIN },
        };

        float old1 = 0.0, 
              old2 = 0.0,
              alt = 0.0;

        while (!thread_should_exit) {
                int ret_sp = poll(fd_sp, 1, 500);
                if (ret_sp < 0) {
			warnx("poll sp error");
		} else if (ret_sp == 0) {
			/* no return value - nothing has happened */
		} else if (fd_sp[0].revents & POLLIN) {
                        orb_copy(ORB_ID(quad_att_sp), quad_sp_sub, &sp);
                } else {
                        /* nothing happened */
                }

                if (sp.cmd == QUAD_ATT_CMD_START) {
                        /* int attConRet = attControl((double)sp.roll, (double)sp.pitch, (double)sp.yaw, (double)sp.thrust); */
                                
                        /* if (attConRet < 0) */
                        /*         mavlink_log_info(mavlink_fd, "[quad_att__control] attControl failed to start"); */
                        
                        int ret_sensor = poll(fds, 1, 250);
                        if (ret < 0) {
                                warnx("poll error");
                        } else if (ret_sensor == 0) {
                                /* no return value - nothing has happened */
                        } else if (fds[0].revents & POLLIN) {
                                orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);
                        }
                        
                        int ret_v_att = poll(fd_v_att, 1, 250);
                        if (ret_v_att < 0) {
                                warnx("poll error");
                        } else if (ret_v_att == 0) {
                                /* no return value - nothing has happened */
                        } else if (fd_v_att[0].revents & POLLIN) {
                                orb_copy(ORB_ID(vehicle_attitude), v_att_sub, &v_att);
                        }
                        
                        
                                                
                        /* check_topic(); */
                } else if (sp.cmd == QUAD_ATT_CMD_STOP) {
                        /* attControl((double)sp.roll, (double)sp.pitch, (double)sp.yaw); */
                        actuators.control[0] = (float)0;
                        actuators.control[1] = (float)0;
                        actuators.control[2] = (float)0;
                        actuators.control[3] = (float)0.0;

                        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                } else {
                        /* Nothhing to do */
                }
        }
}

float filterMA(float *old1, float *old2, float new) {
        int N = 3,
            res = 0;
        res = (*old1 + *old2 + new)/N;
        *old2 = *old1;
        *old1 = res;
        return res;
}

/* int controller() { */
        
/* if ( (double)rel_alt < (double)0.8 ) { */
/*         att_sp.thrust += (float)0.2; */
/*         att_sp.roll_body = 0; */
/*         att_sp.pitch_body = 0; */
/*         att_sp.yaw_body = 0; */
/*         orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp); */
/*         printf("alt skru op: %8.4f\n", (double)att_sp.thrust); */
/*         printf("alt sensor: %8.4f\n\n", (double)rel_alt); */
/* } else if ((double)rel_alt > (double)0.8 ) { */
/*         att_sp.thrust -= (float)0.2; */
/*         att_sp.roll_body = 0; */
/*         att_sp.pitch_body = 0; */
/*         att_sp.yaw_body = 0; */
/*         att_sp.q_d_valid = true; */
/*         orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp); */
/*         printf("alt skru ned: %8.4f\n", (double)att_sp.thrust); */
/*         printf("alt sensor: %8.4f\n\n", (double)rel_alt); */
/* } else { */
/*         /\* this is bad *\/ */
/* } */
/*         return 0; */
/* } */

int attControl(float roll, float pitch, float yaw, float thrust) {
        float resRoll = 0.0, 
              resPitch = 0.0, 
              resYaw = 0.0, 
              resThrust = 0.0;

        

        actuators.control[0] = resRoll;
        actuators.control[1] = resPitch;
        actuators.control[2] = resYaw;
        actuators.control[3] = resThrust;

        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
        return 0;
}

static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: att_control {start|stop|status}\n\n");
        exit(1);
}

int quad_att_control_main(int argc, char *argv[]) {
        if (argc < 1)
		usage("missing argument");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("att_control already running\n");

			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("att_control",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 20,
                                             2048,
                                             att_control_thread_main,
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
			printf("\tatt_control is running\n");

		} else {
			printf("\tatt_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
