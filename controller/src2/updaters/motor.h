#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

#include "encoder.h"
#include "controller.h"
#include "raw_motor.h"

class Motor : public BaseStateUpdater<MotorState> {
    private:
        Encoder encoder;
        Controller controller;
        RawMotor raw_motor;
    public:
        Motor(MotorState& state) : BaseStateUpdater<MotorState>(state), encoder(Encoder(state)), controller(Controller(state)), raw_motor(RawMotor(state)) {}
        void update(Tick& tick) {
            encoder.update(tick);
            controller.update(tick);
            raw_motor.update(tick);
        }
};