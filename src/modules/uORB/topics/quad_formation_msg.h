/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/** 
 * Topic created to handle messages from groundstation
 * to the quad_formation_control application.
 */

#ifndef QUAD_FORMATION_MSG
#define QUAD_FORMATION_MSG

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

enum QUAD_MSG_CMD {
        QUAD_MSG_CMD_START = 42,
	QUAD_MSG_CMD_STOP = 43,
	QUAD_MSG_CMD_TEST = 44,
	QUAD_MSG_CMD_ENUM_END = 45
};

struct quad_formation_msg_s {
        float x;           ///< x-axis
        float y;           ///< y-axis
        float z;           ///< z-axis -1 if coordinate is Not Available
        uint8_t target_system;   ///<  The target_system is defined in enum QUAD_FORMATION_ID
        uint8_t cmd_id;          ///< Command ID is defined in enum QUAD_CMD
        uint8_t pos_no;          ///< Coordinate set number
        uint64_t timestamp;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(quad_formation_msg);

#endif
