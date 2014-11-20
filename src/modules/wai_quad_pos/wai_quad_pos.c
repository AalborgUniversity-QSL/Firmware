/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/**
 * Implementation of an quadrotor formation control app for use in the 
 * semester project.
 */
 
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>

#include <systemlib/param/param.h>

#include <mavlink/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/quad_formation_msg.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_vicon_position.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
 
__EXPORT int wai_quad_pos_main(int argc, char *argv[]);
int wai_quad_pos_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
// struct quad_formation_msg_s update_quad_topic(void);


static bool thread_running = false;
static bool thread_should_exit = false;
static int daemon_task;

int wai_quad_pos_thread_main(int argc, char *argv[]){

        static int mavlink_fd;

        struct sensor_combined_s raw;
        memset(&raw, 0, sizeof(raw));
        struct quad_formation_msg_s pos;
        memset(&pos, 0, sizeof(pos));
        struct vehicle_status_s st;
        memset(&st, 0, sizeof(st));
        // struct vehicle_vicon_position_s vicon;
        // memset(&vicon, 0, sizeof(vicon));

        warnx("[wai] Started ");
        mavlink_log_info(mavlink_fd,"[wai] Started");
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

        int alt_sub = orb_subscribe(ORB_ID(sensor_combined)); 
        int vhe_sub = orb_subscribe(ORB_ID(vehicle_status));
        int quad_sub = orb_subscribe(ORB_ID(quad_formation_msg));
        // int vicon_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));

        orb_set_interval(alt_sub,100);

        // uint64_t last_run = 0; 
        // float t_diff = 0; 
        int package_loss = 0;

        struct pollfd fds[1];
        fds[0].fd = quad_sub;
        fds[0].events = POLLIN;

        while (!thread_should_exit) {

                int pret = poll(fds, 1, 1000);

                if (pret < 0) {
                        warnx("poll cmd error");
                } else if (pret == 0){
                        package_loss++;
                        if(package_loss > 10){
                                package_loss = 0;
                                printf("Connection is bad");
                        }

                } else {
                        if (fds[0].revents & POLLIN) {
                                
                                orb_copy(ORB_ID(quad_formation_msg), quad_sub, &pos);

                                /*Test sample rate*/
                                /*t_diff = (pos.timestamp - last_run)/1000000.0f;
                                last_run = pos.timestamp;
                                printf("rate: %.3f \n",(double)t_diff);*/

                                bool vehicle_status_updated;
                                orb_check(vhe_sub, &vehicle_status_updated);

                                if (vehicle_status_updated){
                                        orb_copy(ORB_ID(vehicle_status), vhe_sub, &st);
                                }

                                bool vehicle_status_updated;
                                orb_check(vhe_sub, &vehicle_status_updated);

                                 if (vehicle_status_updated){ 
                                        orb_copy(ORB_ID(vehicle_status), vhe_sub, &st);
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
                                             SCHED_PRIORITY_MAX - 6,
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
