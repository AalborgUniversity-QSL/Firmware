/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/

/** 
 * Topic created to handle messages from groundstation
 * to the qua_formation_control application.
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

struct quad_formation_msg_s {
        int16_t x[10];           ///< x-axis (int16_t[10])
        int16_t y[10];           ///< y-axis (int16_t[10])
        int16_t z[10];           ///< z-axis (int16_t[10]) = -1 if coordinate is Not Available
        uint8_t target_system;   ///<  The target_system is defined in enum QUAD_FORMATION_ID
        uint8_t cmd_id;          ///< Command ID is defined in enum QUAD_CMD
        uint8_t pos_no;          ///< Coordinate set number
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(quad_formation_msg);

#endif
