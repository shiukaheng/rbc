#pragma once

struct MotorPinout {
    int lpwm_pin;
    int rpwm_pin;
    int hall_a_pin;
    int hall_b_pin;
};

struct MotorState {
    MotorPinout motor_pinout;
    double gear_ratio;
    int ppr;
    int smoothener_window_size;
    int min_startup_pwm;
};

struct MotorState : MotorState {
    double kp;
    double ki;
    double kd;
};

struct RBCConfig {
    MotorState motor_configs[4];
};