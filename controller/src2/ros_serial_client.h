#include "clock.h"

struct Motor {
    // Hardware configuration
    const int lpwm_pin;
    const int rpwm_pin;
    const int hall_a_pin;
    const int hall_b_pin;
    const double gear_ratio;
    const int ppr;
    const double wheel_radius;
    const double wheel_x;
    const double wheel_y;
    const double wheel_theta;
    const double wheel_gamma;
    // Controller settings
    double p_in = 0;
    double i_in = 0;
    double d_in = 0;
    double bias = 50;
    double windup_min = -255;
    double windup_max = 255;
    double output_min = -255;
    double output_max = 255;
    double deadband_min = 0;
    double deadband_max = 0;
    // Controller state
    double i_accumulator = 0;
    // Encoder settings
    double max_abs_acceleration = 200.;
    double max_abs_speed = INFINITY;
}

class ROSSerialClient {
    private:
        long last_update_time;
        Clock clock;
        // Vector of motors
        std::vector<Motor> motors;
    public:
        void update() {
            Tick tick = clock.update();

        }
};