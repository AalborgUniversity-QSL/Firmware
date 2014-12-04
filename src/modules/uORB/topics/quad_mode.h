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
        QUAD_CMD_TAKEOFF = 12,
	QUAD_CMD_SWARM = 13,
	QUAD_CMD_LAND = 14,
	QUAD_CMD_GROUNDED = 15
};

struct quad_mode_s {
        uint8_t cmd;
};

#endif
