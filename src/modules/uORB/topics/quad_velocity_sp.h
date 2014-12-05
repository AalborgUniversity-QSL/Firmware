/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/** 
 * Topic created to handle messages from swarm control application
 * to the quad_att_control application.
 */

#ifndef QUAD_VELOCITY_SP_H
#define QUAD_VELOCITY_SP_H
 
#include <stdint.h>
#include "../uORB.h"
 
struct quad_velocity_sp_s {
        uint64_t timestamp;
        float roll;    /* vector for sp */
        float pitch;
        float thrust;
};
 
ORB_DECLARE(quad_velocity_sp);
 
#endif
