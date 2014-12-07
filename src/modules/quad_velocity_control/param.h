#ifndef PARAM
#define PARAM

float	Kp_thrust = 0.2,//0.00008,
        Kd_thrust = 0.11, /* Controller constants for thrust controller */
        Kp_pos = 0.06,
        Kd_pos = 0.01, /* Controller constants for position controller */
 	
 	hover_alt = 1,		// 1 meter altitude
 	landing_alt = 0.2,
	hover_threashold = 0.1,
	out_thrust_old = 0,
	anti_gravity = 0.48,
	min_rotor_speed = 0.25,
	speed_up_time = 4,
	min_hover_velocity = 0.001,

	thrust_filter = 0.05;

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
	float ddx;
	float ddy;
	float x_old;
	float y_old;
	float dx_old;
	float dy_old;
	float thrust;
	float thrust_der;
	float thrust_old;
};

#endif