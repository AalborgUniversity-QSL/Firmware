/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************
 *
 * Controller constants for the attutude controllers
 *
 */

#ifndef QUAD_ATT_CONTROL_MAIN_H
#define QUAD_ATT_CONTROL_MAIN_H

struct PD_object_s {
        float desired; //< set point
        float error; //< error
        float prevError; //< previous error
        float deriv; //< derivative
        float outP; //< proportional output
        float outD; //< derivative output
        float kp; //< proportional gain
        float kd; //< derivative gain
        float dt; //< delta-time dt
};

struct output_s {
        float roll;
        float pitch;
        float yaw;
        float thrust;
};

#endif  /* QUAD_ATT_CONTROL_MAIN_H */
