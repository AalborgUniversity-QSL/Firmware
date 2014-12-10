#ifndef QUAD_VELOCITY_CONTROL_PARAM_H
#define QUAD_VELOCITY_CONTROL_PARAM_H

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
	float x_int;
	float x_int_old;
	float y_int;
	float y_int_old;
	float x_old;
	float y_old;
	float z_old;
	float dx_old;
	float dy_old;
	float thrust;
	float thrust_der;
	float thrust_int;
	float thrust_int_old;
	float thrust_old;
};

#endif  /* QUAD_VELOCITY_CONTROL_PARAM_H */
