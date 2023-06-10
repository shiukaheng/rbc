#pragma once

#include <Arduino.h>

#define PPR 16
#define GEAR_RATIO 1./82.

class EncoderReaderNaive{
    public:
        EncoderReaderNaive(int hall_a_pin, int hall_b_bin, int ppr = PPR, double gear_ratio = GEAR_RATIO, long min_dt = 10000, long max_dt = 50000) {
            _hall_a_pin = hall_a_pin;
            _hall_b_pin = hall_b_bin;
            _ppr = ppr;
            _gear_ratio = gear_ratio;
            _min_dt = min_dt;
            _max_dt = max_dt;
            pinMode(_hall_a_pin, INPUT);
            pinMode(_hall_b_pin, INPUT);
        }
        ~EncoderReaderNaive();
        /**
         * @brief The ISR hook that should be triggered by the A pin of the encoder
         * 
         */
        void isr() {
            // See if hall_b_pin is high or low
            bool cw = digitalRead(_hall_b_pin);
            // Increment or decrement the total count
            if (cw) {
                total_count++;
                delta_count++;
            } else {
                total_count--;
                delta_count--;
            }
        }
        /**
         * @brief Updates the monitor
         * 
         */
        void update() {
            // Get the time
            long now = micros();
            // Get the delta time
            long dt = now - _last_update_time;
            // If the delta time is too small, return
            // Slow speed: higher _min_dt is better, Higher speed: lower _min_dt is better. Me bound this to max_dt, min_dt to guarantee 
            // Target: PPR amount of pulses per second
            // Expected window time to capture PPR amount (1 cycle) of pulses at wheel rot vel w: gear ratio / w
            double expected_dt = constrain(_radps == 0 ? _max_dt : _gear_ratio / _radps * 1e6, _min_dt, _max_dt);
            // double expected_dt = _max_dt;
            if (dt < expected_dt) {
                return;
            }
            // Get the delta pulses
            long delta_pulses = delta_count;
            // Reset the delta count
            delta_count = 0;
            // Calculate the rad/s
            _radps = (double) delta_pulses / _ppr * _gear_ratio / dt * 1e6 * 2 * M_PI;
            // Update the last update time
            _last_update_time = now;
            // Update the cumulative rad using total_count
            _cumulative_rad = (double) total_count / _ppr * _gear_ratio * 2 * M_PI;
        }
        /**
         * @brief Gets the rad/s
         * 
         * @return double The rad/s
         */
        double getRPS() {
            return _radps;
        }
        /**
         * @brief Get the cumulative radians
         * 
         * @return double The cumulative radians
         */
        double getCumulativeRad() {
            return _cumulative_rad;
        }
    private:
        long _last_update_time = 0;
        double _radps = 0;
        double _cumulative_rad = 0;
        // Params
        int _hall_a_pin;
        int _hall_b_pin;
        int _ppr;
        double _gear_ratio;
        long _min_dt;
        long _max_dt;
        // ISR vars
        volatile long total_count = 0;
        volatile long delta_count = 0;
};