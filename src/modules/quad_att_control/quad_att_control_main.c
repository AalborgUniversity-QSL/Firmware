/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************
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
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/quad_velocity_sp.h>
#include <uORB/topics/vehicle_attitude.h>

#include <geo/geo.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <lib/mathlib/mathlib.h>

#include "quad_att_control_main.h"
#include "params.h"

/* Function prototypes */
__EXPORT int quad_att_control_main(int argc, char *argv[]);
int att_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
void pd_init(struct PD_object_s *pd, float desired, float kp, float kd, float dt);
void pd_set_dt(struct PD_object_s* pd, float dt);
float pd_update(struct PD_object_s* pd, float measured);
void pd_set_desired(struct PD_object_s* pd, float desired);

/**
 * Globals
 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


int att_control_thread_main(int argc, char *argv[]) {
        warnx("[quad_att_control] has begun");

        static int mavlink_fd;
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
        mavlink_log_info(mavlink_fd, "[quad_att_control] started");

        struct output_s out;
        memset(&out, 0, sizeof(out));
        struct PD_object_s roll;
        memset(&roll, 0, sizeof(roll));
        struct PD_object_s pitch;
        memset(&pitch, 0, sizeof(pitch));
        struct PD_object_s yaw;
        memset(&yaw, 0, sizeof(yaw));

        /**
         * Subscriptions
         */
        struct quad_velocity_sp_s sp;
        memset(&sp, 0, sizeof(sp));
        struct vehicle_attitude_s v_att;
        memset(&v_att, 0, sizeof(v_att));

        int sp_sub = orb_subscribe(ORB_ID(quad_velocity_sp));
        int v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

        /**
         * Topics to be published on
         */
        struct actuator_controls_s actuators;
        memset(&actuators, 0, sizeof(actuators));

        orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

        struct pollfd fd_v_att[1];
        fd_v_att[0].fd = v_att_sub;
        fd_v_att[0].events = POLLIN;

        pd_init(&roll, (float)0, (float)ROLL_PITCH_KP, (float)ROLL_PITCH_KD, (float)0.01);
        pd_init(&pitch, (float)0, (float)ROLL_PITCH_KP, (float)ROLL_PITCH_KD, (float)0.01);
        pd_init(&yaw, (float)0, (float)YAW_KP, (float)YAW_KD, (float)0.01);

        float   abs_yaw = 0.f,
                dt = 0.f,
                time = 0.f,
                time_old = 0.f,
                rp_max = 0.7,  /* roll and pitch maximum output */
                yaw_max = 0.4;  /* yaw maximum output */

        while ( !thread_should_exit ) {
                int ret_v_att = poll(fd_v_att, 1, 1);
                if (ret_v_att < 0) {
                        warnx("poll sp error");
                } else if (ret_v_att == 0) {
                        /* no return value - nothing has happened */
                } else if (fd_v_att[0].revents & POLLIN) {
                        orb_copy(ORB_ID(vehicle_attitude), v_att_sub, &v_att);

                        bool sp_updated;
                	orb_check(sp_sub, &sp_updated);

	                if ( sp_updated ){
	                        orb_copy(ORB_ID(quad_velocity_sp), sp_sub, &sp);
	                }

                        abs_yaw = fabs(v_att.yaw);
                        v_att.yaw = (float)-1 * ((PI - abs_yaw) * (v_att.yaw / abs_yaw)); /* Correct the yaw angle to be about zero */

                        pd_set_desired(&roll, sp.roll);
                        pd_set_desired(&pitch, sp.pitch);
                        pd_set_desired(&yaw, sp.yaw);

                        time = (hrt_absolute_time() / (float)1000000);
                        dt = time - time_old;
                        time_old = time;

                        pd_set_dt(&roll, dt);
                        pd_set_dt(&pitch, dt);
                        pd_set_dt(&yaw, dt);

                        out.roll = pd_update(&roll, v_att.roll);
                        out.pitch = pd_update(&pitch, v_att.pitch);
                        out.yaw = pd_update(&yaw, v_att.yaw);

                        /* Limiting attitude controllers output */
                        if ( (float)fabs(out.roll) > rp_max )
                                out.roll = rp_max * (out.roll / (float)fabs(out.roll));

                        if ( (float)fabs(out.pitch) > rp_max )
                                out.pitch = rp_max * (out.pitch / (float)fabs(out.pitch));

                        if ( (float)fabs(out.yaw) > yaw_max )
                                out.yaw = yaw_max * (out.yaw / (float)fabs(out.yaw));

                        out.thrust = sp.thrust; /* Thrust controller resides in velocity controller */

                        actuators.control[0] = (float)out.roll;
                        actuators.control[1] = (float)out.pitch;
                        actuators.control[2] = (float)out.yaw;
                        actuators.control[3] = (float)out.thrust;

                        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

                } else {
                        /* nothint happened */
                }
        }
}

float pd_update(struct PD_object_s* pd, float measured) {
        float output;

        pd->error = pd->desired - measured;

        pd->deriv = (pd->error - pd->prevError) / pd->dt;

        pd->outP = pd->kp * pd->error;
        pd->outD = pd->kd * pd->deriv;

        output = pd->outP + pd->outD;

        pd->prevError = pd->error;

        return output;
}

void pd_set_dt(struct PD_object_s* pd, float dt) {
        pd->dt = dt;
}

void pd_set_desired(struct PD_object_s* pd, float desired) {
        pd->desired = desired;
}

void pd_init(struct PD_object_s* pd, float desired, float kp, float kd, float dt) {
        pd->error = 0;
        pd->prevError = 0;
        pd->deriv = 0;
        pd->desired = desired;
        pd->kp = kp;
        pd->kd = kd;
        pd->dt = dt;
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
