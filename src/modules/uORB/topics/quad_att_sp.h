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

/**
 * @addtogroup topics
 * @{
 */

enum QUAD_ATT_CMD {
        QUAD_ATT_CMD_START = 16,
        QUAD_ATT_CMD_STOP = 17
};
 
struct quad_att_sp_s {
        uint64_t timestamp;
        float x;    /* vector for sp */
        float y;
        float z;
        float q[4]; /* quaternion setpoint for quaternion based control */
        float roll;
        float pitch;
        float yaw;
        float thrust;
        uint8_t CMD;
};


/**
 * @}
 */
 
ORB_DECLARE(quad_att_sp);
 
#endif
