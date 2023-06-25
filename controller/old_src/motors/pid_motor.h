#pragma once

#include <Arduino.h>
#include "../ardupid/ArduPID.h"
#include "encoder_naive.h"
#include "raw_motor.h"
#include "../utils/argtypes.h"
#include "../utils/derivative_constrainer.h"

// Minimum PWM to move the motor: 50

// #define MIN_MOVING_PWM 11 // Assumes symmetric PWMs

class Motor {
    public:
        Motor(MotorState config);
        ~Motor();
        /**Encoder
         * @brief Directly set the PWM value of the motor
         * 
         * @param pwm_value the pwm_value to set the motor to [-255, 255]
         */
        void setPWMRaw(int pwm_value);
        /**
         * @brief Set the PWM value of the motor with smoothening as a safety measure
         * 
         * @param pwm_value the pwm_value to set the motor to [-255, 255]
         */
        void setPWM(int pwm_value);
        /**
         * @brief Set the rad/s of the motor (with gearing ratio corrected)
         * 
         * @param rps the rad/s to set the motor to
         */
        void setRPS(float rps);
        /**
         * @brief Updates the motor, should be called in the main loop
         * 
         */
        void update();
        void isr();
        double getRPS();
        double getCumulativeRad();
        MotorState motor_config;
        // PID functions and variables
        ArduPID& getPID();
        EncoderReaderNaive& getEncoder();
        double setpoint;
        double input;
        double output;
    private:
        // Internal PID variables
        ArduPID _pid_controller;
        EncoderReaderNaive* _encoder;
        ExpTimeSmoothener* _stall_smoothener;
        bool _stall_prevention_triggered = false;
        // Motor parameters
        int _lpwm_pin; // Left pwm pin
        int _rpwm_pin; // Right pwm pin
        int _hall_a_pin; // Meant to be attached to interrupt
        int _hall_b_pin; // Read in update function
        double _gear_ratio;
        double _quantization_error;
        // Motor
        RawMotor* _motor;
        // Derivative constrainer
        DerivativeConstrainer* _derivative_constrainer;
};