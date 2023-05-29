#pragma once

#include <Arduino.h>
#include "../motors/pid_motor.h"
#include "../motors/encoder.h"
#include "../motors/raw_motor.h"
#include "../utils/argtypes.h"

/**
 * @brief A client that listens for PID RPM targets, and uses closed loop control to achieve them
 * 
 */
class SerialPIDClient {
    private:
        String _inputBuffer;
        PIDMotor* _motor;
    public:
        SerialPIDClient(PIDMotorConfig config);
        ~SerialPIDClient();
        void update();
        void isr();
};

/**
 * @brief A client that listens for PWM targets, and uses open loop control to achieve them
 * 
 */
class SerialPWMClient {
    private:
        String _inputBuffer;
        EncoderReader* _encoderReader;
        RawMotor* _motor;
    public:
        SerialPWMClient(MotorConfig config);
        ~SerialPWMClient();
        void update();
        void isr();
};