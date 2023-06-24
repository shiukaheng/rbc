#pragma once

#include <Arduino.h>
#include "../motors/pid_motor.h"
#include "../motors/encoder.h"
#include "../motors/raw_motor.h"
#include "../utils/argtypes.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <robocock/WheelStates.h>
#include <robocock/TargetWheelVelocities.h>
#include <robocock/WheelPIDParameters.h>
#include <robocock/WheelAccumulatedI.h>

/**
 * @brief A client that listens for PID RPS targets, and uses closed loop control to achieve them
 *
 */
class RobotCore {
    private:
        Motor* _motor1;
        Motor* _motor2;
        Motor* _motor3;
        Motor* _motor4;
        ros::NodeHandle _nh;
        robocock::WheelStates _wheel_states_msg;
        robocock::TargetWheelVelocities _target_wheel_states_msg;
        ros::Publisher _wheel_states_pub = ros::Publisher("wheel_states", &_wheel_states_msg);
        void _targetWheelVelocitiesCallback(const robocock::TargetWheelVelocities& msg);
        ros::Subscriber<robocock::TargetWheelVelocities, RobotCore> _target_wheel_velocities_sub = ros::Subscriber<robocock::TargetWheelVelocities, RobotCore>("target_wheel_velocities", &RobotCore::_targetWheelVelocitiesCallback, this);
        void _wheelPIDParametersCallback(const robocock::WheelPIDParameters& msg);
        ros::Subscriber<robocock::WheelPIDParameters, RobotCore> _wheel_pid_parameters_sub = ros::Subscriber<robocock::WheelPIDParameters, RobotCore>("wheel_pid_parameters", &RobotCore::_wheelPIDParametersCallback, this);
        void _wheelAccumulatedICallback(const robocock::WheelAccumulatedI& msg);
        ros::Subscriber<robocock::WheelAccumulatedI, RobotCore> _wheel_accumulated_i_sub = ros::Subscriber<robocock::WheelAccumulatedI, RobotCore>("wheel_i_accum", &RobotCore::_wheelAccumulatedICallback, this);
        bool disable_motors_flag;
        float _setpoints[4] = {0,0,0,0};
    public:
        RobotCore(RBCConfig config);
        ~RobotCore();
        void update();
        void isr1();
        void isr2();
        void isr3();
        void isr4();
        void emergencyStop();
        void resume();
        void resetPID();
};