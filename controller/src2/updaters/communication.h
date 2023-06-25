#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"
#include "../states/robot_state.h"

#include "motor.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <robocock/WheelStates.h>
#include <robocock/TargetWheelVelocities.h>
#include <robocock/WheelPIDParameters.h>
#include <robocock/WheelAccumulatedI.h>

class Communication : public BaseStateUpdater<RobotState> {
    private:
        // Create the message objects
        robocock::WheelStates wheel_states_msg;
        robocock::TargetWheelVelocities target_wheel_states_msg;

        // Wheel state publisher
        ros::Publisher wheel_states_pub = ros::Publisher("wheel_states", &wheel_states_msg);

        // Target wheel velocities subscriber
        void setpointCallback(const robocock::TargetWheelVelocities& msg) {
        }
        ros::Subscriber<robocock::TargetWheelVelocities, Communication> target_wheel_velocities_sub = ros::Subscriber<robocock::TargetWheelVelocities, Communication>("target_wheel_velocities", &Communication::setpointCallback, this);
        
        // Wheel PID parameters subscriber
        void parametersCallback(const robocock::WheelPIDParameters& msg) {
        }
        ros::Subscriber<robocock::WheelPIDParameters, Communication> wheel_pid_parameters_sub = ros::Subscriber<robocock::WheelPIDParameters, Communication>("wheel_pid_parameters", &Communication::parametersCallback, this);
        
        // Wheel accumulated I subscriber
        void wheelAccumulatedICallback(const robocock::WheelAccumulatedI& msg);
        ros::Subscriber<robocock::WheelAccumulatedI, Communication> wheel_accumulated_i_sub = ros::Subscriber<robocock::WheelAccumulatedI, Communication>("wheel_i_accum", &Communication::wheelAccumulatedICallback, this);
    public:
        Communication(RobotState& state) : BaseStateUpdater<RobotState>(state) {
            static_assert(sizeof(state.motors) / sizeof(state.motors[0]) == 4, "RobotState::motors must be of length 4"); // If we change motor numbers we need to manually rewrite this class!
        }
        void update(Tick& tick) {
        }
};