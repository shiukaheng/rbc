#pragma once
#include "Arduino.h"

/**
 * This class is a custom adaptive PID controller that has been heavily
 * modified to work well for our system with the observation that PWM is 
 * roughly proportional to the measured speed of the motor.
 * 
 * We originally used the ArduPID library, but this was rewritten for better
 * readability, and to allow for more runtime configuration.
 * 
 * 
 * Compared to the ArduPID implementation, our code does the following differently:
 * 
 * 
 * *** Bias term is multiplied with the sign of the setpoint ***
 * 
 * I interpret this as adding the minimum PWM value to get the motor to start moving
 * 
 * 
 * *** The accumulated integral value is multiplied with the setpoint before being added to the output ***
 * 
 * Out motor has a fast response time and little inertia, intuitively it makes sense that at setpoint 0,
 * the integral term should be zero as well. 
 * 
 * Also makes use of the assumption that the motor speed is roughly proportional to the PWM value.
 * Effectively, this means that the controller converges on creating a coefficient that forms a linear
 * relationship between the setpoint and the measured value.
 * 
 * Under this new scheme, the I term becomes the main contributor to the output. The controller would be able 
 * to work even without the P and D terms (purely adaptive feedforward control), but with the addition of the two 
 * terms, the controller is able to converge faster and with less overshoot. Moreover, the controller is able to
 * adapt to changes in the system, such as a change in the load on the motor.
 * 
 * However, note that with this modification, the I term contributes a lot more especially when the setpoint is high.
 * A smaller value should be chosen for stability. The P term should be tuned similarly to a normal PID controller.
 *
 * In regular PID systems, the I term accumulator is usually initialized to 0. However in our system this represents
 * the linear relationship between the setpoint and the measured value. Therefore, if it is initialized to 0, the
 * controller takes a long time to slowly converge on the correct value. To speed up the process, the accumulator
 * should be initialized to a value that is close to the correct value. Perhaps having persistent storage of the
 * accumulator value would be useful. It could be implemented by storing the value in EEPROM, and also would be useful
 * for ROS to be able to change the accumulator value.
*/
class RBCAdaptivePID {
    public:
        // The following are all properties that can be changed during runtime
        double setpoint = 0; // The desired value
        double p_in = 0; // The P term
        double i_in = 0; // The I term
        double i_accumulator = 0; // The I term accumulator
        double d_in = 0; // The D term
        double bias = 0; // The bias term
        double windup_min = -INFINITY; // The minimum value of the I term accumulator
        double windup_max = INFINITY; // The maximum value of the I term accumulator
        double output_min = -INFINITY; // The minimum output value
        double output_max = INFINITY; // The maximum output value
        double deadband_min = 0; // The minimum error value for the controller to start working
        double deadband_max = 0; // The maximum error value for the controller to start working

        /**
         * Calculates the output of the controller
         * @param input The measured value
         * @param dt The time since the last update in seconds
         * @return The output of the controller
        */
        double update(double input, double dt) {
            double i = i_in * dt;
            double d = d_in / dt;
            double current_input = current_input;
        }

    private:
        double current_input = 0; // The measured value
        double last_input; // The previous measured value, used for calculating the D term
        double output; // The output value of the controller
        double current_error; // The current error value
        double last_error; // The previous error value, used for trapezoidal integration of the I term
};