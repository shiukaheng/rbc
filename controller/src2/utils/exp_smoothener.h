#pragma once
#include <Arduino.h>

class ExpSmoothener {
public:
    /**
     * Construct an ExpTimeSmoothener object.
     *
     * @param decay A double representing the decay rate. A value closer to 1 will retain more past data,
     *              while a value closer to 0 will react more to recent data.
     */
    ExpSmoothener(double& decay) : _decay(decay), _first(true) {}

    /**
     * Update the value to be smoothed.
     *
     * @param value The new raw data value to be included in the smoothed sequence.
     * @param dt The time since the last update, in seconds.
     * @return The smoothed value.
     */
    double update(double value, double dt) {
        // Calculate the decay factor, such that the value decays by _decay in 1 second
        double decay = pow(_decay, dt);

        if (_first) {
            _first = false;
            _last_value = value;
        } else {
            // Apply the decay factor
            _last_value = _last_value * decay + value * (1 - decay);
        }
        return _last_value;
    }

private:
    double _last_value;  // The last smoothed value
    double& _decay;  // The decay rate
    bool _first;  // A flag indicating whether this is the first value in the sequence
};
