#pragma once
#include <Arduino.h>

/**
 * @file physical_constrainer.h
 * @brief Physical constrainer class
 * 
 * This class constrains the acceleration of a system
 * It can be used for:
 * - Filtering out extreme misreadings from sensors by constraining the physically possible acceleration
 * - Constraining the acceleration of a system to prevent damage
*/
class DerivativeConstrainer {
    public:
        DerivativeConstrainer(double max_abs_slope) {
            _max_abs_slope = abs(max_abs_slope);
        }
        ~DerivativeConstrainer();
        double update(double x, double dt) {
            if (_first_run) {
                _first_run = false;
                _last_constrained_x = x;
                return x;
            } else {
                // Calculate the derivative
                double dx = (x - _last_constrained_x) / dt;
                double abs_dx = abs(dx);
                float sign = dx / abs_dx;
                if (abs_dx > _max_abs_slope) {
                    // Constrain the derivative
                    dx = _max_abs_slope * sign;
                }
                // Calculate the constrained x  
                double constrained_x = _last_constrained_x + dx * dt;
                // Update the last constrained x
                _last_constrained_x = constrained_x;
                return constrained_x;
            }
        }
    private:
        double _max_abs_slope;
        bool _first_run = true; // First run flag
        double _last_constrained_x; // Last constrained value of x
};