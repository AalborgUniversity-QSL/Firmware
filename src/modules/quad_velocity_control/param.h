#ifndef PARAM
#define PARAM

float 	formation_alt = 1,		// 1 meter altitude
	altitude_threashold = 0.1,
	anti_gravity = 0.48;


struct init_pos_s {
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