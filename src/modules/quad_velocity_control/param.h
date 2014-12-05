#ifndef PARAM
#define PARAM

float	Kp_thrust = 0.0002,//0.00008,
        Kd_thrust = 0.00011, /* Controller constants for thrust controller */
 	
 	formation_alt = 1,		// 1 meter altitude
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
	float dx;
	float dy;
	float z;
	float thrust;
	float thrust_der;
	float thrust_old;
};

#endif