/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/*
 Topic created to handle command messages from ground station.
 */

#ifndef QUAD_SWARM_CMD
#define QUAD_SWARM_CMD

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/* register this as object request broker structure */

enum QUAD_MSG_CMD {
        QUAD_MSG_CMD_START_SWARM = 42,
	QUAD_MSG_CMD_STOP_SWARM = 43,
	QUAD_MSG_CMD_TAKEOFF = 44,
	QUAD_MSG_CMD_LAND = 45,
	QUAD_MSG_CMD_ENUM_END = 46
};

struct quad_swarm_cmd_s {
	uint8_t target_system;
	uint8_t cmd_id;			// Commands given by ENUM QUAD_MSG_CMD
};

ORB_DECLARE(quad_swarm_cmd);

#endif
