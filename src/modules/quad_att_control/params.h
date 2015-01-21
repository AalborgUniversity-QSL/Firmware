/*************************************************************************
 * Copyright (c) 2014 Group 731 Aalborg University. All rights reserved.
 * Author:   Group 731 <14gr731@es.aau.dk>
 *************************************************************************
 *
 * Controller constants for the attutude controllers
 *
 */

#ifndef ATTITUDE_CONTROL_PARAMS_H
#define ATTITUDE_CONTROL_PARAMS_H

#define QUAD_ROLL_PITCH_KP 0.23
#define QUAD_ROLL_PITCH_KI 0
#define QUAD_ROLL_PITCH_KD 0.05
#define QUAD_ROLL_PITCH_INTEGRATION_LIMIT 20.0

#define QUAD_YAW_KP 0.3
#define QUAD_YAW_KI 0.02
#define QUAD_YAW_KD 0.15
#define QUAD_YAW_INTEGRATION_LIMIT 30.0

#define DEFAULT_PID_INTEGRATION_LIMIT 5000.0

#endif  /* ATTITUDE_CONTROL_PARAMS_H */
