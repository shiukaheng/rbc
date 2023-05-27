#pragma once
#include "ros/ros.h"
#include "serial/serial.h"

class SerialHelper {
    public:
        // Constructor with serial port, baud rate is argument
        SerialHelper(std::string serial_port, int baud_rate);
        ~SerialHelper();
        // Write wheel velocities to serial port
        void write(float wheel1, float wheel2, float wheel3, float wheel4);
        // Read from serial port using callback, reading parsed four wheel velocities
        void setCallback(std::function<void(float, float, float, float)> callback);
    private:
        // Serial port object
        serial::Serial ser;
        // Callback function
        std::function<void(float, float, float, float)> callback;
};