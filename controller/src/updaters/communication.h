#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"
#include "../states/robot_state.h"

#include "motor.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <robocock/BaseState.h>
#include <robocock/BaseSetpoint.h>
#include <robocock/BaseParameters.h>
#include <robocock/BaseAdaptiveState.h>

#include "../defs.h"

/**
 * This class communicates the internal controller states and parameters to ROS via rosserial
 */
class Communication : public BaseStateUpdater<RobotState> {
    private:
        // Create node handle
        ros::NodeHandle nh;

        // Create the message objects
        robocock::BaseState base_state_msg;
        robocock::BaseSetpoint base_setpoint_msg;

        // Wheel state publisher
        ros::Publisher base_state_publisher = ros::Publisher("base_state", &base_state_msg);

        void baseSetpointCallback(const robocock::BaseSetpoint& msg) {
            nh.loginfo("Received base setpoint");
            for (int i = 0; i < NUM_MOTORS; i++) {
                state.motors[i].setpoint = msg.setpoints[i].velocity;
            }
        }
        ros::Subscriber<robocock::BaseSetpoint, Communication> base_setpoint_sub = ros::Subscriber<robocock::BaseSetpoint, Communication>("base_setpoint", &Communication::baseSetpointCallback, this);

        // void baseParametersCallback(const robocock::BaseParameters& msg) {
        //     nh.loginfo("Received base parameters");
        //     state.motors[0].p_in = 50;
        //     for (int i = 0; i < NUM_MOTORS; i++) {
        //         state.motors[i].p_in = msg.parameters[i].p_in;
        //         state.motors[i].i_in = msg.parameters[i].i_in;
        //         state.motors[i].d_in = msg.parameters[i].d_in;
        //         state.motors[i].bias = msg.parameters[i].bias;
        //         // state.motors[i].i_accumulator_min = msg.parameters[i].i_accumulator_min;
        //         // state.motors[i].i_accumulator_max = msg.parameters[i].i_accumulator_max;
        //         // state.motors[i].output_min = msg.parameters[i].output_min;
        //         // state.motors[i].output_max = msg.parameters[i].output_max;
        //         // // state.motors[i].deadband_min = msg.parameters[i].deadband_min;
        //         // // state.motors[i].deadband_max = msg.parameters[i].deadband_max;
        //         // state.motors[i].target_update_rate = msg.parameters[i].target_update_rate;
        //         // state.motors[i].max_abs_acceleration = msg.parameters[i].max_abs_acceleration;
        //         // state.motors[i].max_abs_velocity = msg.parameters[i].max_abs_velocity;
        //         // state.motors[i].second_order_predictor = msg.parameters[i].second_order_predictor;
        //         // state.motors[i].smoothener_alpha = msg.parameters[i].smoothener_alpha;
        //     }
        // }
        // ros::Subscriber<robocock::BaseParameters, Communication> base_parameters_sub = ros::Subscriber<robocock::BaseParameters, Communication>("base_parameters", &Communication::baseParametersCallback, this);
        
        // void baseAdaptiveStateCallback(const robocock::BaseAdaptiveState& msg) {
        //     nh.loginfo("Received adaptive state");
        //     for (int i = 0; i < NUM_MOTORS; i++) {
        //         state.motors[i].i_accumulator = msg.adaptive_states[i].i_accumulator;
        //     }
        // }
        // ros::Subscriber<robocock::BaseAdaptiveState, Communication> base_adaptive_state_sub = ros::Subscriber<robocock::BaseAdaptiveState, Communication>("base_adaptive_state", &Communication::baseAdaptiveStateCallback, this);
    public:
        Communication(RobotState& state) : BaseStateUpdater<RobotState>(state) {
            nh.getHardware()->setBaud(BAUD_RATE);
            nh.initNode();
            nh.subscribe(base_setpoint_sub);
            // nh.subscribe(base_parameters_sub);
            // nh.subscribe(base_adaptive_state_sub);
            nh.advertise(base_state_publisher);
        }
        void update(Tick& tick) {
            // Publish the wheel state
            for (int i = 0; i < NUM_MOTORS; i++) {
                base_state_msg.states[i].i_accumulator = state.motors[i].i_accumulator;
                base_state_msg.states[i].output = state.motors[i].output;
                base_state_msg.states[i].error = state.motors[i].error;
                base_state_msg.states[i].delta_ticks = state.motors[i].delta_ticks;
                base_state_msg.states[i].position = state.motors[i].position;
                base_state_msg.states[i].velocity = state.motors[i].velocity;
                base_state_msg.states[i].acceleration = state.motors[i].acceleration;
            }
            base_state_publisher.publish(&base_state_msg);
            nh.spinOnce();
        }
};