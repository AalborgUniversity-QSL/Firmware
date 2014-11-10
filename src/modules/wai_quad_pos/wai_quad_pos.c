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

 #include <mavlink/mavlink_log.h>

 #include <uORB/uORB.h>
 #include <uORB/topics/sensor_combined.h>
 #include <uORB/topics/quad_formation_msg.h>

 #include <systemlib/systemlib.h>
 
__EXPORT int wai_quad_pos_main(int argc, char *argv[]);
int wai_quad_pos_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static bool thread_running = false;
static bool thread_should_exit = false;
static bool init_pos_set = false;
static int daemon_task;

float z_baro_ajusted;

int16_t x_init [10];
int16_t y_init [10];
int16_t z_init [10];

 
int wai_quad_pos_thread_main(int argc, char *argv[])
{
	warnx("[wai] Started ");
	static int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE,0);

	int qmsg_sub_fd = orb_subscribe(ORB_ID(quad_formation_msg));
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	orb_set_interval(qmsg_sub_fd, 100);
	orb_set_interval(sensor_sub_fd,100);

	struct pollfd fd[] = {
                { .fd = qmsg_sub_fd,   .events = POLLIN },
                { .fd = sensor_sub_fd, .events = POLLIN },
        };

	int error_counter = 0;

	while(true) {
		// wait for sensor update of 2 descriptor for 1000 ms
		int poll_ret = poll(fd, 2, 1000);

		if (poll_ret == 0) {
			mavlink_log_info(mavlink_fd, "[wai@mavlink] No pos recived"); /* Send string with mavlink */
		} 
		else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[wai] ERROR return value from poll(): %d\n", poll_ret);
			}
			error_counter++;
		} 
		else {
			// Update raw Sensor values to the struct raw
			if (fd[1].revents & POLLIN) {
				struct sensor_combined_s raw;
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

				z_baro = (float)raw.baro_alt_meter;
				z_baro_ajusted = z_baro;

				if(!init_pos_set && qmsg.cmd_id == QUAD_MSG_CMD_START) {
					
				}
			}

			// Update Quadrotor position from vicon data
			if (fd[0].revents & POLLIN) {
				struct quad_formation_msg_s qmsg;
				orb_copy(ORB_ID(quad_formation_msg), qmsg_sub_fd, &qmsg);

				if(!init_pos_set) {
					// (int16_t)z_init = ()
				}

				mavlink_log_info(mavlink_fd,"[wai@mavlink] sample no: %u ([%d \t %d \t %d]) \n",
						(uint8_t)qmsg.pos_no,
						(int16_t)qmsg.x[0],
						(int16_t)qmsg.y[0],
						(int16_t)qmsg.z[0]);

				if(init_pos_set) {
					// Do WHO AM I algorithm
				} 
				
			}
		}
	}

	return 0;
}



// User interface for running in shell
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