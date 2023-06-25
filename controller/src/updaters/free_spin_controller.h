#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

#include "../utils/exp_smoothener.h"

class FreeSpinAddonController : public BaseStateUpdater<MotorState> {
    public:
        double vel_smoothener_alpha = 0.95;
        double acc_smoothener_alpha = 0.95;
        double decay_rate = 1; // Percent of velocity to decay per second
        ExpSmoothener vel_exp_smoothener = ExpSmoothener(vel_smoothener_alpha);
        ExpSmoothener acc_exp_smoothener = ExpSmoothener(acc_smoothener_alpha);
        double velocity_gain = 1;
        FreeSpinAddonController(MotorState& state) : BaseStateUpdater<MotorState>(state) {}
        void update(Tick& tick) {
            // Normalize decay rate using dt
            if (acc_exp_smoothener.update(state.acceleration, tick.dt) < 6) { // If motor is spinning on its own, decay
                decay_rate = 0.8;
            } else { // If motor is being accelerated, don't decay
                decay_rate = 0;
                velocity_gain = 1;
            }
            velocity_gain = 1 - decay_rate * tick.dt;
            state.setpoint = vel_exp_smoothener.update(state.velocity, tick.dt) * velocity_gain; 
        }
};