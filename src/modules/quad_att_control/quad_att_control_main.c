/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************
 *
 * Implementation of an quadrotor attitude control app for use in the
 * semester project.
 *
 */

#include <stdio.h>
#include <poll.h>
#include <mavlink/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/quad_mode.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/quad_velocity_sp.h>
#include <uORB/topics/vehicle_attitude.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>

#include "quad_att_control_main.h"
#include "params.h"

/* Function prototypes */
__EXPORT int quad_att_control_main(int argc, char *argv[]);
int att_control_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
void pid_init(struct PID_object_s *pid, float desired, float kp, float ki, float kd, float dt);
void pid_set_dt(struct PID_object_s *pid, float dt);
float pid_update(struct PID_object_s *pid, float measured);
void pid_set_desired(struct PID_object_s *pid, float desired);
void pid_set_integral_limit(struct PID_object_s *pid, float limit);
void pid_set_integral_limit_low(struct PID_object_s *pid, float limitLow);

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
        struct PID_object_s roll;
        memset(&roll, 0, sizeof(roll));
        struct PID_object_s pitch;
        memset(&pitch, 0, sizeof(pitch));
        struct PID_object_s yaw;
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
        struct quad_mode_s mode;
        memset(&mode, 0, sizeof(mode));

        orb_advert_t mode_pub = orb_advertise(ORB_ID(quad_mode), &mode);
        orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

        struct pollfd fd_v_att[1];
        fd_v_att[0].fd = v_att_sub;
        fd_v_att[0].events = POLLIN;

        pid_init(&roll, 0, QUAD_ROLL_PITCH_KP, QUAD_ROLL_PITCH_KI, QUAD_ROLL_PITCH_KD, 0.01);
        pid_init(&pitch, 0, QUAD_ROLL_PITCH_KP, QUAD_ROLL_PITCH_KI, QUAD_ROLL_PITCH_KD, 0.01);
        pid_init(&yaw, 0, QUAD_YAW_KP, QUAD_YAW_KI, QUAD_YAW_KD, 0.01);

        pid_set_integral_limit(&roll, QUAD_ROLL_PITCH_INTEGRATION_LIMIT);
        pid_set_integral_limit(&pitch, QUAD_ROLL_PITCH_INTEGRATION_LIMIT);
        pid_set_integral_limit(&yaw, QUAD_YAW_INTEGRATION_LIMIT);

        pid_set_integral_limit_low(&roll, QUAD_ROLL_PITCH_INTEGRATION_LIMIT);
        pid_set_integral_limit_low(&pitch, QUAD_ROLL_PITCH_INTEGRATION_LIMIT);
        pid_set_integral_limit_low(&yaw, QUAD_YAW_INTEGRATION_LIMIT);

        float   abs_yaw = 0.f,
                dt = 0.f,
                time = 0.f,
                time_old = 0.f,
                rp_safe = 0.3,
                rp_max = 0.7,  /* roll and pitch maximum output */
                yaw_max = 0.4;  /* yaw maximum output */

        bool error = false;

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

                        pid_set_desired(&roll, sp.roll);
                        pid_set_desired(&pitch, sp.pitch);
                        pid_set_desired(&yaw, sp.yaw);

                        time = (hrt_absolute_time() / (float)1000000);
                        dt = time - time_old;
                        time_old = time;

                        pid_set_dt(&roll, dt);
                        pid_set_dt(&pitch, dt);
                        pid_set_dt(&yaw, dt);

                        out.roll = pid_update(&roll, v_att.roll);
                        out.pitch = pid_update(&pitch, v_att.pitch);
                        out.yaw = pid_update(&yaw, v_att.yaw);

                        /* Limiting attitude controllers output */
                        if ( (float)fabs(out.roll) > rp_max )
                                out.roll = rp_max * (out.roll / (float)fabs(out.roll));

                        if ( (float)fabs(out.pitch) > rp_max )
                                out.pitch = rp_max * (out.pitch / (float)fabs(out.pitch));

                        if ( (float)fabs(out.yaw) > yaw_max )
                                out.yaw = yaw_max * (out.yaw / (float)fabs(out.yaw));

                        out.thrust = sp.thrust; /* Thrust controller resides in velocity controller */

                        if ( (v_att.roll > rp_safe) || (v_att.pitch > rp_safe) || (error == true) ) {
                                out.roll = 0;
                                out.pitch = 0;
                                out.yaw = 0;
                                out.thrust = 0;

                                mode.error == true;
                                orb_publish(ORB_ID(quad_mode), mode_pub, &mode);

                                error = true;
                        }

                        actuators.control[0] = (float)out.roll;
                        actuators.control[1] = (float)out.pitch;
                        actuators.control[2] = (float)out.yaw;
                        actuators.control[3] = (float)out.thrust;

                        orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

                } else {
                        /* nothing happened */
                }
        }
}

float pid_update(struct PID_object_s* pid, float measured) {
        float output;

        pid->error = pid->desired - measured;

        pid->integ += pid->error * pid->dt;
        if ( pid->integ > pid->iLimit ) {
                pid->integ = pid->iLimit;
        }
        else if ( pid->integ < pid->iLimitLow ) {
                pid->integ = pid->iLimitLow;
        }

        pid->deriv = (pid->error - pid->prevError) / pid->dt;

        pid->outP = pid->kp * pid->error;
        pid->outI = pid->ki * pid->integ;
        pid->outD = pid->kd * pid->deriv;

        output = pid->outP + pid->outI + pid->outD;

        pid->prevError = pid->error;

        return output;
}

void pid_set_dt(struct PID_object_s* pid, float dt) {
        pid->dt = dt;
}

void pid_set_desired(struct PID_object_s* pid, float desired) {
        pid->desired = desired;
}

void pid_set_integral_limit(struct PID_object_s *pid, float limit) {
        pid->iLimit = limit;
}

void pid_set_integral_limit_low(struct PID_object_s *pid, float limitLow) {
        pid->iLimitLow = -limitLow;
}

void pid_init(struct PID_object_s *pid, float desired, float kp, float ki, float kd, float dt) {
        pid->error     = 0;
        pid->prevError = 0;
        pid->integ     = 0;
        pid->deriv     = 0;
        pid->desired = desired;
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
        pid->iLimit    = (float)DEFAULT_PID_INTEGRATION_LIMIT;
        pid->iLimitLow = (float)DEFAULT_PID_INTEGRATION_LIMIT * (float)-1.0;
        pid->dt        = dt;
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
