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
#include <uORB/topics/quad_formation_msg.h>
#include <uORB/topics/vehicle_attitude.h>

#include <geo/geo.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

struct attError_s {
        float roll;
        float pitch;
        float yaw;
        float thrust;
};

struct output_s {
        float roll;
        float pitch;
        float yaw;
        float thrust;
};

struct pos_s {
        float x;
        float y;
        float z;
};

struct pos_error_s {
        float x;
        float y;
        float z;
};

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
        struct quad_att_sp_s sp;
        memset(&sp, 0, sizeof(sp));
        struct vehicle_attitude_s v_att;
        memset(&v_att, 0, sizeof(v_att));
        struct quad_formation_msg_s qmsg;
        memset(&qmsg, 0, sizeof(qmsg));

        int quad_sp_sub = orb_subscribe(ORB_ID(quad_att_sp));
        int v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
        int qmsg_sub = orb_subscribe(ORB_ID(quad_formation_msg));

        /* Published */
        struct actuator_controls_s actuators;
        memset(&actuators, 0, sizeof(actuators));
        
        for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
                actuators.control[i] = 0.0f;
        }
        orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

        struct pollfd fd_sp[1];
        fd_sp[0].fd = quad_sp_sub;
        fd_sp[0].events = POLLIN;

        struct pollfd fd_v_att[1];
        fd_v_att[0].fd = v_att_sub;
        fd_v_att[0].events = POLLIN;

        struct attError_s error;
        memset(&error, 0, sizeof(error));
        struct output_s out;
        memset(&out, 0, sizeof(out));
        struct attError_s error_old;
        memset(&error_old, 0, sizeof(error_old));
        struct attError_s error_der;
        memset(&error_der, 0, sizeof(error_old));
        struct attError_s v_att_offset;
        memset(&v_att_offset, 0, sizeof(v_att_offset));
        struct pos_s pos_offset;
        memset(&pos_offset, 0, sizeof(pos_offset));
        struct pos_error_s pos_error;
        memset(&pos_error, 0, sizeof(pos_error));
        
        float   Kp = 0.14,
                Kd = 0.017,
                Kp_yaw = 0.08,//0.018,
                Kd_yaw = 0.12,
                Kp_thrust = 0.000025,
                Kd_thrust = 0.000040,
                // Kp_pos = 0.00006,//125,
                // Kd_pos = 0.0001,
                dt = 0.01,
                dt_z = 0.1,
                anti_gravity = 0.45,
                error_thrust_der = 0,
                error_thrust_old = 0,
                // error_x_der  = 0,
                // error_x_old = 0,
                // error_y_der = 0,
                // error_y_old = 0,
                abs_yaw = 0;

        bool first = true;

        while (!thread_should_exit) {
                int ret_sp = poll(fd_sp, 1, 1);
                if (ret_sp < 0) {
                        warnx("poll sp error");
                } else if (ret_sp == 0) {
                        /* no return value - nothing has happened */
                } else if (fd_sp[0].revents & POLLIN) {
                        orb_copy(ORB_ID(quad_att_sp), quad_sp_sub, &sp);
                        /* printf("yes, vi fik et SP poll\n"); */
                } else {
                        /* nothing happened */
                }

                if (sp.cmd == (enum QUAD_MSG_CMD)QUAD_ATT_CMD_START) {

                        int ret_v_att = poll(fd_v_att, 1, 1);
                        if (ret_v_att < 0) {
                                warnx("poll sp error");
                        } else if (ret_v_att == 0) {
                                /* no return value - nothing has happened */
                        } else if (fd_v_att[0].revents & POLLIN) {
                                orb_copy(ORB_ID(vehicle_attitude), v_att_sub, &v_att);

                                abs_yaw = fabs(v_att.yaw);
                                v_att.yaw = (float)-1 * (((float)3.141592 - abs_yaw) * (v_att.yaw / abs_yaw));

                                bool qmsg_updated;
                                orb_check(qmsg_sub, &qmsg_updated);
                                if (qmsg_updated) {
                                        orb_copy(ORB_ID(quad_formation_msg), qmsg_sub, &qmsg);

                                        if ( first == true ) {
                                                v_att_offset.yaw = v_att.yaw;
                                                pos_offset.x = qmsg.x;
                                                pos_offset.y = qmsg.y;
                                                first = false;
                                        }

                                        error.thrust = sp.z - qmsg.z;

                                        // if ( error.thrust > (float)1000 ) {
                                        //         error.thrust = 1000;
                                        // } else if ( error.thrust < (float)0 ) {
                                        //         error.thrust = 0;
                                        // }

                                        error_thrust_der = (error.thrust - error_thrust_old)/dt_z;
                                        out.thrust = (float)Kp_thrust * (float)error.thrust + (float)Kd_thrust * (float)error_thrust_der + anti_gravity;
                                        error_thrust_old = error.thrust;
                                        
                                        if ( out.thrust > (float)1 ) {
                                                out.thrust = (float)1;
                                        } else if ( out.thrust < anti_gravity ) {
                                                out.thrust = anti_gravity;
                                        }
                                        
                                        // pos_error.x = pos_offset.x - qmsg.x;
                                        // pos_error.y = pos_offset.y - qmsg.y;
                                        
                                        // error_x_der = (pos_error.x - error_x_old)/dt_z;
                                        // error_y_der = (pos_error.y - error_y_old)/dt_z;

                                        // error_x_old = pos_error.x;
                                        // error_y_old = pos_error.y;
                                }
                       
                                error.roll = sp.roll - v_att.roll;
                                error.pitch = sp.pitch - v_att.pitch;
                                error.yaw = sp.yaw -v_att.yaw + v_att_offset.yaw;

                                error_der.roll = (error.roll - error_old.roll)/dt;
                                error_der.pitch = (error.pitch - error_old.pitch)/dt;
                                error_der.yaw = (error.yaw - error_old.yaw)/dt;

                                error_old.roll = error.roll;
                                error_old.pitch = error.pitch;
                                error_old.yaw = error.yaw;

                                out.roll =  /*- Kp_pos * pos_error.y - Kd_pos * error_y_der + */(float)Kp * (float)error.roll + Kd * error_der.roll;
                                out.pitch = /*- Kp_pos * pos_error.x - Kd_pos * error_x_der + */(float)Kp * (float)error.pitch + Kd * error_der.pitch;
                                out.yaw = (float)Kp_yaw * (float)error.yaw + Kd_yaw * error_der.yaw;

                                if ( out.roll > (float)1 ) {
                                        out.roll = 1.f;
                                } else if ( out.roll < (float)-1 ) {
                                        out.roll = -1;
                                }

                                if ( out.pitch > (float)1 ) {
                                        out.pitch = 1.f;
                                } else if ( out.pitch < (float)-1 ) {
                                        out.pitch = -1;
                                }

                                if ( out.yaw > (float)1 ) {
                                        out.yaw = 1.f;
                                } else if ( out.yaw < (float)-1 ) {
                                        out.yaw = -1;
                                }
                       
                        } else {
                                /* nothing happened */
                        }       

                } else if (sp.cmd == (enum QUAD_MSG_CMD)QUAD_ATT_CMD_STOP) {
                        out.thrust = 0.f;
                        out.roll = 0.f;
                        out.pitch = 0.f;
                        out.yaw = 0.f;

                } else {
                        /* Nothhing to do */
                }
                actuators.control[0] = (float)out.roll;
                actuators.control[1] = (float)out.pitch;
                actuators.control[2] = (float)out.yaw;
                actuators.control[3] = (float)out.thrust;
                /* mavlink_log_info(mavlink_fd, "[quad_att] y:%.3f", (double)v_att.yaw); */
                // mavlink_log_info(mavlink_fd, "[quad_att] x:%.3f y:%.3f z:%.3f", (double)qmsg.x, (double)qmsg.y, (double)qmsg.z);
                // mavlink_log_info(mavlink_fd, "[quad_att] r:%.3f p:%.3f yaw:%.3f", (double)v_att.roll, (double)v_att.pitch, (double)v_att.yaw); 
		/* mavlink_log_info(mavlink_fd, "[quad_att] r:%.3f p:%.3f y:%.3f T:%.3f", (double)out.roll, (double)out.pitch, (double)out.yaw, (double)out.thrust);  */

                orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
        }
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
                                             SCHED_PRIORITY_MAX - 5,
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
