#pragma once
#include "rbc.h"

// Function to create the RBCConfig
RBCConfig createConfig(int motor_pinouts[4][4], int ppr, double gear_ratio, int smoothener_window_size, double kp, double ki, double kd, int min_startup_pwm) {
    // Concatenate the motor pinouts and pid into PIDMotorConfigs
    RBCConfig config;
    for (int i = 0; i < 4; i++) {
        MotorPinout motor_pinout;
        motor_pinout.lpwm_pin = motor_pinouts[i][0];
        motor_pinout.rpwm_pin = motor_pinouts[i][1];
        motor_pinout.hall_a_pin = motor_pinouts[i][2];
        motor_pinout.hall_b_pin = motor_pinouts[i][3];
        config.motor_configs[i].motor_pinout = motor_pinout;
        config.motor_configs[i].ppr = ppr;
        config.motor_configs[i].gear_ratio = gear_ratio;
        config.motor_configs[i].smoothener_window_size = smoothener_window_size;
        config.motor_configs[i].kp = kp;
        config.motor_configs[i].ki = ki;
        config.motor_configs[i].kd = kd;
        config.motor_configs[i].min_startup_pwm = min_startup_pwm;
    }
    return config;
}