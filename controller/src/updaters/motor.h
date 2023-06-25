#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

#include "encoder.h"
#include "controller.h"
#include "raw_motor.h"
#include "free_spin_controller.h"

/**
 * Motor class that contains all the sub-updaters for closed-loop control
 */
class Motor : public BaseStateUpdater<MotorState> {
    public:
        Encoder encoder;
        Controller controller;
        RawMotor raw_motor;
        FreeSpinAddonController free_spin_controller;
        
        Motor(MotorState& state) : BaseStateUpdater<MotorState>(state), encoder(Encoder(state)), controller(Controller(state)), raw_motor(RawMotor(state)), free_spin_controller(FreeSpinAddonController(state)) {}
        void update(Tick& tick) {
            encoder.update(tick);
            free_spin_controller.update(tick);
            controller.update(tick);
            raw_motor.update(tick);
        }
};