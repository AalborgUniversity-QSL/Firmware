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

int wai_quad_pos_thread_main(int argc, char *argv[]){

        static bool init_pos_set = false;

        static int max_no_of_quads = 10;
        static int no_of_quads = 10;                    // Initial guess
        static int mavlink_fd;
        static int MA_order = 10;
        static int min_error_no = 0;

        float z_baro_ajust = 0;
        float z_baro;
        float alt_diff[max_no_of_quads];
        float SMA[10] = {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.};
        float z_SMA = 0;
        float alt_detect_threshold = 1;
        float z_zero_quad;

        float init_pos_x, init_pos_y, init_pos_z;
        float z_zero[10];

        struct quad_formation_msg_s qmsg;
        struct sensor_combined_s raw;
        struct vehicle_status_s state;

        warnx("[wai] Started ");

        mavlink_fd = open(MAVLINK_LOG_DEVICE,0);

        int qmsg_sub_fd = orb_subscribe(ORB_ID(quad_formation_msg));
        int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
        int state_sub_fd = orb_subscribe(ORB_ID(vehicle_status));

        // orb_copy(ORB_ID(quad_formation_msg), qmsg_sub_fd, &qmsg);
        // orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
        // orb_copy(ORB_ID(vehicle_status), state_sub, &state);

        orb_set_interval(qmsg_sub_fd, 100);
        orb_set_interval(sensor_sub_fd,100);

        struct pollfd fd[] = {
                { .fd = qmsg_sub_fd,   .events = POLLIN },
                { .fd = sensor_sub_fd, .events = POLLIN },
                { .fd = state_sub_fd, .events = POLLIN },
        };

        int error_counter = 0;

        while(true) {
                // wait for sensor update of 2 descriptor for 1000 ms
                int poll_ret = poll(fd, 2, 1000);

                if (poll_ret == 0) {
                        mavlink_log_info(mavlink_fd, "[wai@mavlink] No pos recived");
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

                                // Find the total no of quads
                                no_of_quads = max_no_of_quads;

                                for (int i = 0; i < max_no_of_quads; ++i){
                                        if((float)qmsg.z[i] == -1){
                                                no_of_quads = no_of_quads - 1;
                                        }
                                }
                        }

                        // Find a matching coordinate set from increasing the altitude of the quad (Waving to point out where I am)
                        if (fd[1].revents & POLLIN) {
                                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);

                                /* Filter the raw barometer data with a Moving Average filter with an order of MA_order */
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
                        }

                        if (fd[2].revents & POLLIN) {
                                orb_copy(ORB_ID(vehicle_status), state_sub_fd, &state);
                        }
                }

                if (init_pos_set) {
                        float pos_error[no_of_quads];

                        for (int i = 0; i < no_of_quads; ++i){
                                pos_error[i] = sqrt(pow((init_pos_x - qmsg.x[i]),2) + pow((init_pos_y - qmsg.y[i]),2) + pow((init_pos_z - qmsg.z[i]),2));
                                
                                if (i > 0 && pos_error[i] < pos_error[i-1]){
                                        min_error_no = i;                                               
                                }
                        }

                        init_pos_x = qmsg.x[min_error_no];
                        init_pos_y = qmsg.y[min_error_no];
                        init_pos_z = qmsg.z[min_error_no] - z_zero_quad;
                }

                if (!init_pos_set){
                        mavlink_log_info(mavlink_fd,"entered alt...")
                        // Update the initial altitude while in standby
                        if (state.arming_state == ARMING_STATE_STANDBY){
                                mavlink_log_info(mavlink_fd,"standby")
                                z_baro_ajust = z_SMA;

                                for (int i = 0; i < no_of_quads; ++i){
                                        z_zero[i] = (float)qmsg.z[i];                                   
                                }
                                mavlink_log_info(mavlink_fd,"[wai] alt_vic:%.3f \t alt_baro:%.3f \t no:%d",(double)z_zero[0],(double)z_baro_ajust,no_of_quads);
                        }

                        else if (qmsg.cmd_id == QUAD_MSG_CMD_START) {

                                // Increase the thrust until the threshold is met (function)

                                if ((z_SMA - z_baro_ajust) >= alt_detect_threshold){

                                        for (int i = 1; i < no_of_quads; ++i){
                                                // Find the minimum difference between the barometer data and the Vicon position data
                                                alt_diff[i] = (z_SMA - z_baro_ajust) - (float)qmsg.z[i];

                                                if(alt_diff[i] < alt_diff[min_error_no]){
                                                        min_error_no = i;
                                                }
                                        }

                                        init_pos_x = (float)qmsg.x[min_error_no];
                                        init_pos_y = (float)qmsg.y[min_error_no];
                                        init_pos_z = (float)qmsg.z[min_error_no];
                                        z_zero_quad = z_zero[min_error_no];

                                        init_pos_set = true;

                                        // mavlink_log_info(mavlink_fd,"[wai] no:%d \t pos: [%.3f,%.3f,%.3f]",no_of_quads,(double)init_pos_x,(double)init_pos_y,(double)init_pos_z);

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