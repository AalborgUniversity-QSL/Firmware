#ifndef PARAM
#define PARAM

float	Kp_thrust = 0.2,//0.00008,
        Kd_thrust = 0.11, /* Controller constants for thrust controller */
 	
 	formation_alt = 1,		// 1 meter altitude
 	landing_alt = 0.1,
	altitude_threashold = 0.1,
	out_thrust_old = 0,
	anti_gravity = 0.48,
	min_rotor_speed = 0.25,
	speed_up_time = 4;

struct init_pos_s {
	float timestamp;
	float init_x;
	float init_y;
	float init_z;
};

struct state_transition_s {
	bool takeoff;
	bool land;
	bool start;
	bool stop;
	bool in_air;
};

struct quad_alt_velocity {
	float timestamp;		// timestamp in seconds
	float x;
	float y;
	float z;
	float dx;
	float dy;
	float x_old;
	float y_old;
	float thrust;
	float thrust_der;
	float thrust_old;
};

#endif