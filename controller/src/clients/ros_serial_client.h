#pragma once

#include <Arduino.h>
#include "../motors/pid_motor.h"
#include "../motors/encoder.h"
#include "../motors/raw_motor.h"
#include "../utils/argtypes.h"
#include <ros.h>
#include <std_msgs/String.h>

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
        float _rpm_targets[4];
        float _draft_rpm_targets[4];
        String _rpm_target_strings[4];
        ros::NodeHandle _nh;
        std_msgs::String _str_msg;
        ros::Publisher _wheel_velocities_pub("wheel_velocities", &_str_msg);
    public:
        ROSSerialClient(RBCConfig config);
        ~ROSSerialClient();
        void update();
        void isr1();
        void isr2();
        void isr3();
        void isr4();
};