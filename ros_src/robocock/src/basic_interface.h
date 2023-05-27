#pragma once
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "robocock/WheelVelocities.h"

class BasicInterface {
    public:
        BasicInterface();
        ~BasicInterface();
        void run();
    private:
        void readParams(); // Read parameters from parameter server (mostly serial port)
        void wheelVelocitiesCallback(const robocock::WheelVelocities::ConstPtr& msg); // Callback for handling incoming wheel velocity commands
        void writeWheelVelocities(const robocock::WheelVelocities::ConstPtr& msg); // Write wheel velocities to serial port
        std::string serial_port;
        ros::NodeHandle n;
        ros::Subscriber sub;
};