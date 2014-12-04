/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/** 
 * Topic created to handle messages from swarm control application
 * to the quad_att_control application.
 */

#ifndef QUAD_ATT_SP_H
#define QUAD_ATT_SP_H
 
#include <stdint.h>
#include "../uORB.h"
 
struct quad_att_sp_s {
        uint64_t timestamp;
        float dx;    /* vector for sp */
        float dy;
        float z;
};
 
ORB_DECLARE(quad_att_sp);
 
#endif
