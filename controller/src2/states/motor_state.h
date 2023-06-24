#pragma once
#include <Arduino.h>

struct MotorState {
    // *** Hardware configuration ***
    const int lpwm_pin;
    const int rpwm_pin;
    const int hall_a_pin;
    const int hall_b_pin;
    const double gear_ratio;
    const int ppr;

    // *** Controller settings ***
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

    // *** Controller state ***
    double i_accumulator = 0;

    // *** Encoder settings ***
    double target_update_rate = 30; // The target update rate in Hz
    double max_abs_acceleration = 200.; // Threshold of acceleration to discard the update
    double max_abs_velocity = INFINITY; // Threshold of velocity to discard the update
    bool second_order_predictor = false; // Whether to use a second order predictor, read encoder source code for more info

    // *** Encoder state ***
    bool discarded = false; // Whether the last update was discarded due to filtering
    // Unfiltered raw values
    long update_time = 0; // The last micros() value when the encoder was updated
    long delta_ticks = 0;  // The number of ticks since the last update
    long total_ticks = 0; // The total number of ticks since the controller was initialized
    // Filtered values
    double position = 0; // The current position in radians
    double velocity = 0; // The current velocity in radians/s
    double acceleration = 0; // The current acceleration in radians/s^2
};