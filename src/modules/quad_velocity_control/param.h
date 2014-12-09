#ifndef PARAM
#define PARAM

struct init_pos_s {
	float timestamp;
	float x;
	float y;
	float z;
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
	float dz;
	float x_old;
	float y_old;
	float z_old;
	float dx_old;
	float dy_old;
	float thrust;
	float thrust_der;
	float thrust_old;
};

#endif