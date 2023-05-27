#include "basic_interface.h"

BasicInterface::BasicInterface() {
    // Initialize serial port to empty string
    serial_port = "";
    readParams();
}

void BasicInterface::readParams() {
    n.getParam("serial_port", serial_port);
    // Check if serial_port is empty
    if (serial_port.empty()) {
        ROS_ERROR("serial_port parameter is empty");
        exit(1);
    }   
}

