/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/
/*
 * @file wai_quad_pos.c
 * 
 * Implementation of an quadrotor formation control app for use in the 
 * semester project.
 *
 */
 
 #include <nuttx/config.h>
 #include <unistd.h>
 #include <stdio.h>
 #include <errno.h>
 #include <poll.h>
 #include <math.h>
 #include <time.h>

 #include <uORB/uORB.h>
 #include <uORB/topics/quad_formation_msg.h>
 
__EXPORT int wai_quad_pos_main(int argc, char *argv[]);
int wai_quad_pos_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
 
int wai_quad_pos_thread_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");

	int qmsg_sub = orb_subscribe(ORB_ID(quad_formation_msg));

	struct pollfd fds[] = {
		{ .fd = qmsg_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
		 
	};

	// int error_counter = 0;

	return OK;
}

static void usage(const char *reason) {
        if (reason)
                fprintf(stderr, "%s\n", reason);

        fprintf(stderr, "usage: wai_quad_pos {start|stop|status}\n\n");
        exit(1);
}

int wai_quad_pos_main(int argc, char *argv[]) {
        if (argc < 1)
		usage("missing argument");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("wai_quad_pos already running\n");

			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("wai_quad_pos",
                                             SCHED_DEFAULT,
                                             SCHED_PRIORITY_MAX - 20,
                                             2048,
                                             wai_quad_pos_thread_main,
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
			printf("\twai_quad_pos is running\n");

		} else {
			printf("\twai_quad_pos not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}