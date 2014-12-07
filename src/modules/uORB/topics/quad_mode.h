/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/** 
 * Topic created to handle messages from groundstation
 * to the quad_formation_control application.
 */

#ifndef QUAD_MODE
#define QUAD_MODE

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/* register this as object request broker structure */
ORB_DECLARE(quad_mode);

enum QUAD_CMD {
        QUAD_CMD_START_SWARM = 42,
	QUAD_CMD_STOP_SWARM = 43,
	QUAD_CMD_TAKEOFF = 44,
	QUAD_CMD_LAND = 45,
	QUAD_CMD_PENDING = 46
};

enum QUAD_STATE{
        QUAD_STATE_SWARMING = 47,
        QUAD_STATE_HOVERING = 48,
        QUAD_STATE_GROUNDED = 49,
        QUAD_STATE_EMERGENCY = 50
};

struct quad_mode_s {
        uint8_t cmd;
        uint8_t current_state;
        bool error;
};

#endif
