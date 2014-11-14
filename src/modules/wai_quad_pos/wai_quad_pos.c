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

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
 
__EXPORT int wai_quad_pos_main(int argc, char *argv[]);
int wai_quad_pos_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
void check_sys_update(int *topic_sub);

static bool thread_running = false;
static bool thread_should_exit = false;
static int daemon_task;

int wai_quad_pos_thread_main(int argc, char *argv[]){

        static int mavlink_fd;

        struct sensor_combined_s alt;
        memset(&alt, 0, sizeof(alt));
        struct quad_formation_msg_s quad;
        memset(&quad, 0, sizeof(quad));

        warnx("[wai] Started ");
        mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

        int alt_sub = orb_subscribe(ORB_ID(sensor_combined));
        int vhe_sub = orb_subscribe(ORB_ID(vehicle_status));
        int quad_sub = orb_subscribe(ORB_ID(quad_formation_msg));


        while (true) {

                struct pollfd fds[] = {
                        { .fd = alt_sub, .events = POLLIN },
                        // { .fd = quad_sub, .events = POLLIN }
                };

                int pret = poll(fds, 1, 1000);
                orb_set_interval(alt_sub, 500);
                // orb_set_interval(quad_sub, 1000);

                if (pret == 0){

                }
                else if (pret < 0) {
                        warnx("poll cmd error");
                }
                else {
                        if (fds[0].revents & POLLIN) {
                                struct sensor_combined_s raw;
                                memset(&raw, 0, sizeof(raw));
                                orb_copy(ORB_ID(sensor_combined), alt_sub, &raw);

                                alt.baro_alt_meter = raw.baro_alt_meter;

                                // mavlink_log_info(mavlink_fd,"alt: %.3f", (double)alt.baro_alt_meter);
                        }
                        // if (fds[1].revents & POLLIN) {
                        //         struct quad_formation_msg_s pos;
                        //         memset(&pos, 0, sizeof(pos));
                        //         orb_copy(ORB_ID(quad_formation_msg), quad_sub, &pos);

                        //         quad.cmd_id = pos.cmd_id;
                        //         mavlink_log_info(mavlink_fd,"alt: %d", quad.cmd_id);

                        // }

                }

                // check_sys_update(&vhe_sub);
                bool updated;
                struct vehicle_status_s st;

                orb_check(vhe_sub, &updated);

                if (updated){
                        orb_copy(ORB_ID(vehicle_status), vhe_sub, &st);
                        // mavlink_log_info(mavlink_fd,"Armed");
                }

                bool updated2;
                struct quad_formation_msg_s pos;

                orb_check(quad_sub, &updated2);

                if (updated2){
                        orb_copy(ORB_ID(quad_formation_msg), quad_sub, &pos);
                        mavlink_log_info(mavlink_fd,"Recived Vicon -> x: %.3f",(double)pos.x);
                }
        }

        return 0;
}

void check_sys_update(int *topic_sub){
        bool updated;
        struct vehicle_status_s st;

        orb_check(*topic_sub, &updated);

        if (updated){
                orb_copy(ORB_ID(vehicle_status), *topic_sub, &st);
                // mavlink_log_info(mavlink_fd,"vehicle state");
        }
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