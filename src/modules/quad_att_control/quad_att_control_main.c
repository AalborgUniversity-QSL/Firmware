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
#include <lib/mathlib/mathlib.h>

#include "quad_att_control_main.h"


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

        float   Kp = 0.23,
                Kd = 0.05, /* Controller constants for roll and pitch controllers */
                Kp_yaw = 0.3,
                Kd_yaw = 0.15,  /* Controller constants for yaw controller */
                Kp_thrust = 0.0002,//0.00008,
                Kd_thrust = 0.00011, /* Controller constants for thrust controller */
                Kp_pos = 0.00006,
                Kd_pos = 0.00001, /* Controller constants for position controller */
                anti_gravity = 0.44, /* Thrust offset */
                min_rotor_speed = 0.25,
                off_set = 0,
                error_thrust_der = 0,
                error_thrust_old = 0,
                out_thrust_old = 0,
                error_x_der  = 0,
                error_x_old = 0,
                error_y_der = 0,
                error_y_old = 0,
                abs_yaw = 0,    /* Constant for use in yaw controller */
                pos_max = 0.1,
                pos_roll = 0,
                pos_pitch = 0,
                rp_max = 0.7,   /* roll and pitch maximum output */
                yaw_max = 0.4,  /* yaw maximum output */
                dt = 0.01,
                dt_z = 0.1,
                time = 0,
                t0 = 0,
                time_old = 0,
                time_att = 0,
                time_att_old = 0;


        bool    first = true,
                output = true;  /* enabling and disabling actuator outputs  */

        // int     n = 0;

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
                        } else if (fd_v_att[0].revents & POLLIN) { /* Inner loop - handles attitude control */
                                orb_copy(ORB_ID(vehicle_attitude), v_att_sub, &v_att);

                                abs_yaw = fabs(v_att.yaw);
                                v_att.yaw = (float)-1 * ((PI - abs_yaw) * (v_att.yaw / abs_yaw)); /* Correct the yaw angle to be about zero */

                                bool qmsg_updated;
                                orb_check(qmsg_sub, &qmsg_updated);
                                if ( qmsg_updated ) { /* Outer loop - handles positional feedback */
                                        orb_copy(ORB_ID(quad_formation_msg), qmsg_sub, &qmsg);

                                        /* Calculating dt for position loop */
                                        time = (hrt_absolute_time() / (float)1000000); /* time is in seconds */
                                        dt_z = time - time_old;

                                        /* mavlink_log_info(mavlink_fd, "[quad_att] delta time:%.3f, rate [Hz]: %.3f", (double)dt_z, (double)(1 / dt_z)); */
                                        time_old = time;

                                        /* Saving offsets (only first loop) */
                                        if ( first == true ) {
                                                v_att_offset.yaw = v_att.yaw;
                                                pos_offset.x = qmsg.x;
                                                pos_offset.y = qmsg.y;
                                                t0 = time;
                                                first = false;
                                        }

                                        /* Calculating thrust error */
                                        error.thrust = sp.z - qmsg.z;

                                        error_thrust_der = (error.thrust - error_thrust_old)/dt_z;

                                        out_thrust_old = out.thrust - (float)0.04;

                                        out.thrust = (float)Kp_thrust * (float)error.thrust + (float)Kd_thrust * (float)error_thrust_der;

                                        if (out.thrust > out_thrust_old + (float)0.01){
                                                out.thrust = out_thrust_old + (float)0.01;
                                        } else if (out.thrust < out_thrust_old - (float)0.01) {
                                                out.thrust = out_thrust_old - (float)0.01;
                                        }

                                        out.thrust += (float)0.04;

                                        error_thrust_old = error.thrust;

                                        if ( out.thrust > (float)1 ) {
                                                out.thrust = (float)1;
                                        } else if ( out.thrust < 0 ) {
                                                out.thrust = 0;
                                        }

                                        if ( (t0 + (float)4) > (float)time ) {
                                                off_set = min_rotor_speed; //anti_gravity = min_rotor_speed;
                                                out.thrust = 0;
                                        } else {
                                                off_set = anti_gravity; //= min_rotor_speed + (float)0.06;
                                        }


                                        /* Calculating position error */
                                        pos_error.x = pos_offset.x - qmsg.x;
                                        pos_error.y = pos_offset.y - qmsg.y;

                                        /* Calculating derivative of position error */
                                        error_x_der = (pos_error.x - error_x_old)/dt_z;
                                        error_y_der = (pos_error.y - error_y_old)/dt_z;

                                        /* Saving position error for next loop */
                                        error_x_old = pos_error.x;
                                        error_y_old = pos_error.y;
                                }

                                if ( ((hrt_absolute_time() / (float)1000000) - time) > (float)0.5 ) {  /* Emergency handling if connection from gnd is lost */
                                        sp.cmd = (enum QUAD_MSG_CMD)QUAD_ATT_CMD_STOP;
                                        goto emergency_shutdown;
                                }

                                if ( fabs(v_att.roll) > 0.4 ||  fabs(v_att.pitch) > 0.4 ){
                                        sp.cmd = (enum QUAD_MSG_CMD)QUAD_ATT_CMD_STOP;
                                        goto emergency_shutdown;
                                }

                                /* Calculating attitude error */
                                error.roll = sp.roll - v_att.roll;
                                error.pitch = sp.pitch - v_att.pitch;
                                error.yaw = sp.yaw - v_att.yaw; /* - v_att_offset.yaw; */

                                /* Calculating dt for attitude loop */
                                time_att = (hrt_absolute_time() / (float)1000000);
                                dt = time_att - time_att_old;
                                time_att_old = time_att;

                                /* Calculating the derivative of the attitude error */
                                error_der.roll = (error.roll - error_old.roll)/dt;
                                error_der.pitch = (error.pitch - error_old.pitch)/dt;
                                error_der.yaw = (error.yaw - error_old.yaw)/dt;

                                /* Saving attitude error for next loop */
                                error_old.roll = error.roll;
                                error_old.pitch = error.pitch;
                                error_old.yaw = error.yaw;

                                /* Calculating position controllers outputs */
                                pos_roll = - Kp_pos * pos_error.y - Kd_pos * error_y_der;
                                pos_pitch = - Kp_pos * pos_error.x - Kd_pos * error_x_der;

                                // n++;
                                // if (n == 100) {
                                //      mavlink_log_info(mavlink_fd, "[quad_att] thrust:%.3f", (double)out.thrust);
                                //      n = 0;
                                // }

                                /* killing position controllers */
                                /* pos_roll = 0;
                                 * pos_pitch = 0; */

                                /* Limiting position controllers output */
                                if ((float)fabs(pos_roll) > pos_max)
                                        pos_roll = pos_max * (pos_roll / (float)fabs(pos_roll));

                                if ((float)fabs(pos_pitch) > pos_max)
                                        pos_pitch = pos_max * (pos_pitch / (float)fabs(pos_pitch));

                                /* Calculate attitude controllers output - and fusing positional controls with attitude controls*/
                                out.roll =  (float)Kp * (float)error.roll + Kd * error_der.roll + pos_roll;
                                out.pitch = (float)Kp * (float)error.pitch + Kd * error_der.pitch + pos_pitch;
                                out.yaw = (float)Kp_yaw * (float)error.yaw + Kd_yaw * error_der.yaw;

                                /* Limiting attitude controllers output */
                                if ( (float)fabs(out.roll) > rp_max )
                                        out.roll = rp_max * (out.roll / (float)fabs(out.roll));

                                if ( (float)fabs(out.pitch) > rp_max )
                                        out.pitch = rp_max * (out.pitch / (float)fabs(out.pitch));

                                if ( (float)fabs(out.yaw) > yaw_max )
                                        out.yaw = yaw_max * (out.yaw / (float)fabs(out.yaw));

                        } else {
                                /* nothing happened */
                        }

                } else if ( sp.cmd == (enum QUAD_MSG_CMD)QUAD_ATT_CMD_STOP ) {
                emergency_shutdown: /* Only used if an emergency arises. Seriously a problem if necessary */
                        out.thrust = 0.f;
                        out.roll = 0.f;
                        out.pitch = 0.f;
                        out.yaw = 0.f;
                        off_set = 0.f;
                        first = true;

                }// else {
                //         anti_gravity = 0;
                // }

                if ( output == true ) {
                        actuators.control[0] = (float)out.roll;
                        actuators.control[1] = (float)out.pitch;
                        actuators.control[2] = (float)out.yaw;
                        actuators.control[3] = (float)out.thrust + (float)off_set;

                        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
                }



                /* mavlink_log_info(mavlink_fd, "[quad_att] y:%.3f", (double)v_att.yaw); */
                // mavlink_log_info(mavlink_fd, "[quad_att] x:%.3f y:%.3f z:%.3f", (double)qmsg.x, (double)qmsg.y, (double)qmsg.z);
                // mavlink_log_info(mavlink_fd, "[quad_att] r:%.3f p:%.3f yaw:%.3f", (double)v_att.roll, (double)v_att.pitch, (double)v_att.yaw);
                // mavlink_log_info(mavlink_fd, "[quad_att] r:%.3f p:%.3f y:%.3f T:%.3f", (double)out.roll, (double)out.pitch, (double)out.yaw, (double)out.thrust);
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
