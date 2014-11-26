/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/** 
 * Topic created to handle messages from groundstation
 * to the quad_formation_control application.
 */

#ifndef QUAD_CMD
#define QUAD_CMD

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/* register this as object request broker structure */
ORB_DECLARE(quad_cmd);

enum QUAD_CMD {
        QUAD_CMD_START = 42,
	QUAD_CMD_STOP = 43,
	QUAD_CMD_TEST = 44,
        QUAD_CMD_LAND = 45,
        QUAD_CMD_EMRGENCY = 46,
	QUAD_CMD_ENUM_END = 47
};

struct quad_cmd_s {
        uint8_t target_system;   ///<  The target_system is defined in enum QUAD_FORMATION_ID
        uint8_t cmd_id;          ///< Command ID is defined in enum QUAD_CMD
        uint8_t pos_no;          ///< Coordinate set number
        uint64_t timestamp;
};

#endif
