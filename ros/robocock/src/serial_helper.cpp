#include "serial_helper.h"

SerialHelper::SerialHelper(std::string serial_port, int baud_rate) {
    // Initialize serial port object
    ser.setPort(serial_port);
    ser.setBaudrate(baud_rate);
    // Open serial port
    ser.open();
    // Check if serial port is open
    if (!ser.isOpen()) {
        ROS_ERROR("Serial port %s is not open", serial_port.c_str());
        exit(1);
    }
    // Start reading from serial port, processing seperated messages by newline
}

SerialHelper::~SerialHelper() {
    // Close serial port
    ser.close();
}

void SerialHelper::write(float wheel1, float wheel2, float wheel3, float wheel4) {
    // Write wheel velocities to serial port in format of "w1,w2,w3,w4<newline>"
    std::string write_string = std::to_string(wheel1) + "," + std::to_string(wheel2) + "," + std::to_string(wheel3) + "," + std::to_string(wheel4) + "\n";
    ser.write(write_string);
}