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
 #include <uORB/topics/vehicle_status.h>

 #include <systemlib/systemlib.h>
 
__EXPORT int wai_quad_pos_main(int argc, char *argv[]);
int wai_quad_pos_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static bool thread_running = false;
static bool thread_should_exit = false;
static int daemon_task;


// int16_t x_init [10];
// int16_t y_init [10];
// int16_t z_init [10];

 
int wai_quad_pos_thread_main(int argc, char *argv[]){

	// static bool init_pos_set = false;

	static int max_no_of_quads = 10;
	static int no_of_quads = 10;			// Initial guess
	static int mavlink_fd;
	static int MA_order = 10;

	float z_baro_ajust = 0;
	float z_baro;
	// float alt_diff [max_no_of_quads];
	float SMA[10] = {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.};
	float z_SMA = 0;

	struct quad_formation_msg_s qmsg;
	struct sensor_combined_s raw;
	struct vehicle_status_s state;

	warnx("[wai] Started ");

	mavlink_fd = open(MAVLINK_LOG_DEVICE,0);

	int qmsg_sub_fd = orb_subscribe(ORB_ID(quad_formation_msg));
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));

	orb_copy(ORB_ID(quad_formation_msg), qmsg_sub_fd, &qmsg);
	orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
	orb_copy(ORB_ID(vehicle_status), state_sub, &state);

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
			// Update Quadrotor position from vicon data
			if (fd[0].revents & POLLIN) {
				orb_copy(ORB_ID(quad_formation_msg), qmsg_sub_fd, &qmsg);

				for (int i = 0; i < max_no_of_quads; ++i){
					if((float)qmsg.z[i] == -1){
						no_of_quads = no_of_quads - 1;
					}

					// float alt_diff[no_of_quads];
				}

				// mavlink_log_info(mavlink_fd,"[wai@mavlink] sample no: %u ([%3.6f \t %3.6f \t %3.6f]) \n",
				// 		(uint8_t)qmsg.pos_no,
				// 		(float)qmsg.x[0],
				// 		(float)qmsg.y[0],
				// 		(float)qmsg.z[0]);

				// if(init_pos_set) {
				// 	// Do WHO AM I algorithm
				// } 
				
			}

			// Find a matching coordinate set from increasing the altitude of the quad (Waving to point out where I am)
			if (fd[1].revents & POLLIN) {


				/* Filter the raw barometer data with a Moving Average filter with an order of MA_order */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				z_baro = (float)raw.baro_alt_meter;
				
				for (int i = ((int)MA_order - 1); i >= 0; --i){
					if(i > 0){
						SMA[i] = SMA[i-1];
					}
					else {
						SMA[i] = z_baro;
					}
				}
				for (int i = 0; i < MA_order; ++i){
					z_SMA = z_SMA + SMA[i];
				}
				z_SMA = z_SMA/(float)MA_order;

				/*--------------------------------------------------------------------------------------*/

				/* read all relevant states */
				orb_copy(ORB_ID(vehicle_status), state_sub, &state);

				if (state.arming_state == ARMING_STATE_STANDBY){
						z_baro_ajust = z_SMA;
					}

				if (!init_pos_set && qmsg.cmd_id == QUAD_MSG_CMD_START) {
					for (int i = 0; i < no_of_quads; ++i){
						// Find the minimum difference between the barometer data and the vicon position data
						alt_diff[i] = (z_SMA - z_baro_ajust) - (float)qmsg.z[i];
					}
				}

				// mavlink_log_info(mavlink_fd,"[wai@mavlink] z_SMA: \t %.6f",(double)z_SMA - (double)z_baro_ajust);
				// mavlink_log_info(mavlink_fd,"[wai@mavlink] z_SMA: \t %.3f",(double)z_SMA - (double)z_baro_ajust);
				// warnx("[wai] z_baro: \t %f",(double)z_SMA - (double)z_baro_ajust);
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