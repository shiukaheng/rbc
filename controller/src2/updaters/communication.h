#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"
#include "../states/robot_state.h"

#include "motor.h"

class Communication : public BaseStateUpdater<RobotState> {
    public:
        Communication(RobotState& state) : BaseStateUpdater<RobotState>(state) {}
        void update(Tick& tick) {
        }
};