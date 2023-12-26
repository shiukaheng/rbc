#pragma once
#include "motor_state.h"
#include "../defs.h"

struct RobotState {
    MotorState motors[NUM_MOTORS];
    uint8_t battery_level = 100;
    uint16_t battery_voltage = 0;
    bool motor_reset_flag = false;
};