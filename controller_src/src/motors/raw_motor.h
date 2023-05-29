#pragma once

#include <Arduino.h>
#include "../utils/smoothener.h"

/**
 * @brief Raw motor class for directly controlling the motor without encoder feedback
 * 
 */
class RawMotor {
    public:
        /**
         * @brief Construct a new Raw Motor object, sets the pins to output
         * 
         * @param lpwm_pin the left pwm pin
         * @param rpwm_pin the right pwm pin
         */
        RawMotor(int lpwm_pin, int rpwm_pin, int smoothener_window_size = 5);
        ~RawMotor();
        /**
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
         * @brief Updates the motor, should be called in the main loop
         * 
         */
        void update();
    private:
        Smoothener* _smoothener;
        int _lpwm_pin;
        int _rpwm_pin;
        int _target_pwm = 0;
        int _actual_pwm = 0;
        bool _use_smoothener = true;
};