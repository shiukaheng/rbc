#pragma once

#include <Arduino.h>
#include "../motors/pid_motor.h"
#include "../motors/encoder.h"
#include "../motors/raw_motor.h"
#include "../utils/argtypes.h"

/**
 * @brief A client that listens for PID RPS targets, and uses closed loop control to achieve them
 * 
 */
class SerialPIDClient2 {
    private:
        String _inputBuffer;
        PIDMotor* _motor1;
        PIDMotor* _motor2;
        PIDMotor* _motor3;
        PIDMotor* _motor4;
        float _rps_targets[4];
        float _draft_rps_targets[4];
        String _rps_target_strings[4];
    public:
        SerialPIDClient2(RBCConfig config);
        ~SerialPIDClient2();
        void update();
        void isr1();
        void isr2();
        void isr3();
        void isr4();
};