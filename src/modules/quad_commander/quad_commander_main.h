/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************/
/*
 * @file parameters.h
 *
 * Quadrotor commander parameters for use in the
 * semester project.
 *
 */

struct attError_s {
        float roll;
        float pitch;
        float yaw;
        float thrust;
};

struct output_s {
        float roll;
        float pitch;
        float yaw;
        float thrust;
};

struct pos_s {
        float x;
        float y;
        float z;
};

struct pos_error_s {
        float x;
        float y;
        float z;
};
