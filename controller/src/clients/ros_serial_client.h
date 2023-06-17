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
#include <robocock/EmergencyStop.h>
#include <robocock/Resume.h>
#include <robocock/ResetPID.h>

/**
 * @brief A client that listens for PID RPS targets, and uses closed loop control to achieve them
 *
 */
class ROSSerialClient {
    private:
        PIDMotor* _motor1;
        PIDMotor* _motor2;
        PIDMotor* _motor3;
        PIDMotor* _motor4;
        ros::NodeHandle _nh;
        robocock::WheelStates _wheel_states_msg;
        robocock::TargetWheelVelocities _target_wheel_states_msg;
        ros::Publisher _wheel_states_pub = ros::Publisher("wheel_states", &_wheel_states_msg);
        void _targetWheelVelocitiesCallback(const robocock::TargetWheelVelocities& msg);
        ros::Subscriber<robocock::TargetWheelVelocities, ROSSerialClient> _target_wheel_velocities_sub = ros::Subscriber<robocock::TargetWheelVelocities, ROSSerialClient>("target_wheel_velocities", &ROSSerialClient::_targetWheelVelocitiesCallback, this);
        void _wheelPIDParametersCallback(const robocock::WheelPIDParameters& msg);
        ros::Subscriber<robocock::WheelPIDParameters, ROSSerialClient> _wheel_pid_parameters_sub = ros::Subscriber<robocock::WheelPIDParameters, ROSSerialClient>("wheel_pid_parameters", &ROSSerialClient::_wheelPIDParametersCallback, this);
        bool disable_motors_flag;
        float _setpoints[4] = {0,0,0,0};
        bool emergencyStopService(robocock::EmergencyStop::Request &req, robocock::EmergencyStop::Response &res);
        bool resumeService(robocock::Resume::Request &req, robocock::Resume::Response &res);
        bool resetPIDService(robocock::ResetPID::Request &req, robocock::ResetPID::Response &res);
    public:
        ROSSerialClient(RBCConfig config);
        ~ROSSerialClient();
        void update();
        void isr1();
        void isr2();
        void isr3();
        void isr4();
        void emergencyStop();
        void resume();
        void resetPID();
};