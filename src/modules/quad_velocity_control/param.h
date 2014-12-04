#ifndef PARAM
#define PARAM

float formation_alt = 1;		// 1 meter altitude


struct init_pos_s {
	uint64_t timestamp;
	float init_x;
	float init_y;
	float init_z;
};

struct state_transsion_s {
	bool takeoff;
	bool land;
	bool start;
	bool stop;
};

#endif