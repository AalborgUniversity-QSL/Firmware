/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/** 
 * Topic created to handle position messages from ground station
 * to the quad_formation_control application.
 */

#ifndef QUAD_POS_MSG
#define QUAD_POS_MSG

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/* register this as object request broker structure */

struct quad_pos_msg_s {
        uint8_t target_system;   //<  The target_system is defined in enum QUAD_FORMATION_ID
        float timestamp;
        float x[3];           //< x-axis
        float y[3];           //< y-axis
        float z[3];           //< z-axis -1 if coordinate is Not Available
};

ORB_DECLARE(quad_pos_msg);

#endif
