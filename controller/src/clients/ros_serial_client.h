#pragma once

#include <Arduino.h>
#include "../motors/pid_motor.h"
#include "../motors/encoder.h"
#include "../motors/raw_motor.h"
#include "../utils/argtypes.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <robocock/WheelVelocities.h>
#include <robocock/TargetWheelVelocities.h>
#include <robocock/WheelPIDParameters.h>

/**
 * @brief A client that listens for PID RPM targets, and uses closed loop control to achieve them
 *
 */
class ROSSerialClient {
    private:
        PIDMotor* _motor1;
        PIDMotor* _motor2;
        PIDMotor* _motor3;
        PIDMotor* _motor4;
        ros::NodeHandle _nh;
        robocock::WheelVelocities _wheel_velocities_msg;
        robocock::TargetWheelVelocities _target_wheel_velocities_msg;
        ros::Publisher _wheel_velocities_pub = ros::Publisher("wheel_velocities", &_wheel_velocities_msg);
        void _targetWheelVelocitiesCallback(const robocock::TargetWheelVelocities& msg);
        ros::Subscriber<robocock::TargetWheelVelocities, ROSSerialClient> _target_wheel_velocities_sub = ros::Subscriber<robocock::TargetWheelVelocities, ROSSerialClient>("target_wheel_velocities", &ROSSerialClient::_targetWheelVelocitiesCallback, this);
        void _wheelPIDParametersCallback(const robocock::WheelPIDParameters& msg);
        ros::Subscriber<robocock::WheelPIDParameters, ROSSerialClient> _wheel_pid_parameters_sub = ros::Subscriber<robocock::WheelPIDParameters, ROSSerialClient>("wheel_pid_parameters", &ROSSerialClient::_wheelPIDParametersCallback, this);
    public:
        ROSSerialClient(RBCConfig config);
        ~ROSSerialClient();
        void update();
        void isr1();
        void isr2();
        void isr3();
        void isr4();
};