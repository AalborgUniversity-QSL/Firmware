#ifndef PARAM
#define PARAM

float formation_alt = 1;		// 1 meter altitude
float altitude_threashold = 0.1;


struct init_pos_s {
	float timestamp;		// timestamp in seconds
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

struct quad_alt_velocity_sp {
	float timestamp;		// timestamp in seconds
	float dx;
	float dy;
	float altitude;
};

#endif