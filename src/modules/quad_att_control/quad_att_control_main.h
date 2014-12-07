struct PD_object_s {
        float desired; //< set point
        float error; //< error
        float prevError; //< previous error
        float deriv; //< derivative
        float outP; //< proportional output
        float outD; //< derivative output
        float kp; //< proportional gain
        float kd; //< derivative gain
        float dt; //< delta-time dt
};

struct output_s {
        float roll;
        float pitch;
        float yaw;
        float thrust;
};
