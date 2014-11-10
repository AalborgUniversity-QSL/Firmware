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
#include <errno.h>
#include <poll.h>
#include <uORB/topics/quad_formation_msg.h>
 
__EXPORT int wai_quad_pos_main(int argc, char *argv[]);
 
int wai_quad_pos_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");

	//int qmsg_sub = orb_subscribe(ORB_ID(quad_formation_msg));

	/*struct pollfd fds[] = {
		{ .fd = qmsg_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 
	};
	*/

	// int error_counter = 0;

	return OK;
}