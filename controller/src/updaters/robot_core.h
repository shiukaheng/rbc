#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"
#include "../states/robot_state.h"

#include "motor.h"
#include "communication.h"
#include "power_manager.h"

#include "../defs.h"

class RobotCore : public BaseStateUpdater<RobotState> {

    public:

        Communication* communication;
        Motor* motors[NUM_MOTORS];
        PowerManager power_manager;
        int motor_reset_pin = 36;
        double motor_reset_duration = 0.05; // Minimum time to keep the motor reset pin LOW (in seconds)
        double motor_reset_time_elapsed = 0; // Time elapsed since the motor reset pin was set LOW (in seconds)
        bool motor_reset_pin_state;
        
        RobotCore(RobotState& state) : BaseStateUpdater<RobotState>(state), power_manager(state) {
            
            // Let's initialize the communication
            communication = new Communication(state);

            // Then initialize all the motors
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i] = new Motor(state.motors[i]);
            }

            // Set HIGH for motor reset pin
            motor_reset_pin_state = true; // Initialize as HIGH
            pinMode(motor_reset_pin, OUTPUT);
            digitalWrite(motor_reset_pin, HIGH);

        }

        ~RobotCore() {

            // Delete the communication
            delete communication;

            // Delete all the motors
            for (int i = 0; i < NUM_MOTORS; i++) {
                delete motors[i];
            }

        }

        void handleMotorReset(Tick& tick) {

            if (state.motor_reset_flag && motor_reset_pin_state) { // If need to reset motors and currently HIGH (not resetting, and requesting reset)
                digitalWrite(motor_reset_pin, LOW); // We set the motor driver reset pin to LOW
                motor_reset_pin_state = false; // Update the state
                motor_reset_time_elapsed = 0; // Reset the time elapsed
                state.motor_reset_flag = false; // Reset the flag
            } else if (motor_reset_time_elapsed >= motor_reset_duration && !motor_reset_pin_state) { // If the reset duration has elapsed and currently LOW (resetting)
                digitalWrite(motor_reset_pin, HIGH);
                motor_reset_pin_state = true; // Update the state
            } else if (!motor_reset_pin_state) { // If currently LOW (resetting)
                motor_reset_time_elapsed += tick.dt; // Add time elapsed
            }
            // Remaining case: Not resetting, and not requesting reset, do nothing
            
        }

        void update(Tick& tick) {

            // Let's first update all the motors
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i]->update(tick);
            }

            // Then update the power manager
            power_manager.update(tick);

            // Then update the communication
            communication->update(tick);

            // Then handle the motor reset
            handleMotorReset(tick);

        }

};