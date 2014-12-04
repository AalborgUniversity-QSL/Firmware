#ifndef PARAM
#define PARAM

float formation_alt = 1;		// 1 meter altitude
float altitude_threashold = 0.1;


struct init_pos_s {
	uint64_t timestamp;
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

#endif