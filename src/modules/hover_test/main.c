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

#include <mavlink/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/quad_att_sp.h>
#include <uORB/topics/quad_formation_msg.h>

#include <geo/geo.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* Function prototypes */
__EXPORT int hover_test_main(int argc, char *argv[]);
int hover_test_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
int printData(int *pqmsg_sub);

/* Global variables */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

int hover_test_thread_main(int argc, char *argv[]) {

        warnx("[hover_test] started\n");

        /* static int mavlink_fd; */
        /* mavlink_fd = open(MAVLINK_LOG_DEVICE, 0); */
        /* mavlink_log_info(mavlink_fd, "[hover_test] startet.\n"); */

        /* struct quad_formation_msg_s qmsg; */
        /* memset(&qmsg, 0, sizeof(qmsg)); */
        /* struct quad_att_sp_s sp; */
        /* memset(&sp, 0, sizeof(sp)); */

        int qmsg_sub = orb_subscribe(ORB_ID(quad_formation_msg));
//        orb_copy(ORB_ID(quad_formation_msg), qmsg_sub, &qmsg);
        printf("1");
        /* orb_advert_t quad_att_sp_pub = orb_advertise(ORB_ID(quad_att_sp), &sp); */

        struct pollfd fd_qmsg[] = {
                { .fd = qmsg_sub,   .events = POLLIN },
        };
//        bool updated;
//        int k = 0;
        while(!thread_should_exit) {

                /* printf("[hover_test] while loop \n"); */
                int ret_qmsg = poll(fd_qmsg, 1, 1000);
                if (ret_qmsg < 0) {
			warnx("poll cmd error");
		} else if (ret_qmsg == 0) {
			printf("[hover_test] nothing received\n");
		} else if (fd_qmsg[0].revents & POLLIN) {
                /* orb_check(qmsg_sub, &updated); */
                /* if (updated){ */
          
                /*         updated = false; */
                /* } */
//                        qmsg.z[0] = 5.0;
                        /* printf("[hover_test] z = %.3f\n", (double)qmsg.z[0]); */

                        printData(&qmsg_sub);

                        /* if (qmsg.cmd_id == 42) { */
                        /*         printf("x: %.3f\t", (double)qmsg.x); */
                        /*         printf("y: %.3f\t", (double)qmsg.y); */
                        /*         printf("z: %.3f\n", (double)qmsg.z); */

                        /*         /\* printf("[hover_test] start\n"); *\/ */
                        /*         /\* sp.cmd = QUAD_ATT_CMD_START; *\/ */
                        /*         /\* orb_publish(ORB_ID(quad_att_sp), quad_att_sp_pub, &sp); *\/ */
                        /* } else if (qmsg.cmd_id == QUAD_MSG_CMD_STOP){ */
                        /*         /\* printf("[hover_test] stop\n"); *\/ */
                        /*         /\* sp.cmd = QUAD_ATT_CMD_STOP; *\/ */
                        /*         /\* orb_publish(ORB_ID(quad_att_sp), quad_att_sp_pub, &sp); *\/ */
                        /* } */
                }
        }
}

int printData(int *pqmsg_sub) {
        struct quad_formation_msg_s qmsg;
        orb_copy(ORB_ID(quad_formation_msg), *pqmsg_sub, &qmsg);

        printf("x: %.3f\n", (double)qmsg.x);
        printf("y: %.3f\n", (double)qmsg.y);
        printf("z: %.3f\n", (double)qmsg.z);
        printf("target_system: %i\n", qmsg.target_system);
        printf("cmd_id: %i\n", qmsg.cmd_id);
        printf("pos_no: %i\n", qmsg.pos_no);
        printf("timestamp: %i\n", qmsg.timestamp);
        return 0;
}

static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: hover_test {start|stop|status}\n\n");
        exit(1);
}

int hover_test_main(int argc, char *argv[]) {
        if (argc < 1)
		usage("missing argument");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("hover test already running\n");

			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("hover_test",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 20,
                                             2048,
                                             hover_test_thread_main,
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
			printf("\thover test is running\n");

		} else {
			printf("\thover_test not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
