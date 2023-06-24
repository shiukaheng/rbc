#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

class RawMotor : public BaseStateUpdater<MotorState> {
    public:
        RawMotor(MotorState& state) : BaseStateUpdater<MotorState>(state) {}
        void update(Tick& tick) {
        }
};