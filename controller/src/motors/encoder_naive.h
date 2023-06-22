#pragma once

#include <Arduino.h>

#define PPR 16
#define GEAR_RATIO 1./82.

class EncoderReaderNaive{
    public:
        EncoderReaderNaive(int hall_a_pin, int hall_b_bin, int ppr = PPR, double gear_ratio = GEAR_RATIO, long target_dt = 30000) {
            _hall_a_pin = hall_a_pin;
            _hall_b_pin = hall_b_bin;
            _ppr = ppr;
            _gear_ratio = gear_ratio;
            _target_dt = target_dt;
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
            dirty = true;
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
        long update() {
            // Get the time
            long now = micros();
            // Get the delta time
            long dt = now - _last_update_time;
            // If the delta time is less than the target dt, return
            if (dt < _target_dt) {
                return -1;
            }
            dirty = false;
            // Get the delta pulses
            long delta_pulses = delta_count;
            long total_pulses = total_count;
            if (dirty) {
                return -1;
            }
            // Reset the delta count
            delta_count = 0;
            // Calculate the rad/s
            _radps = (double) delta_pulses / (double) _ppr * (double) _gear_ratio / (double) dt * 1e6 * 2. * M_PI;
            // Update the last update time
            _last_update_time = now;
            // Update the cumulative rad using total_count
            _cumulative_rad = (double) total_pulses / (double) _ppr * (double) _gear_ratio * 2. * M_PI;
            // _cumulative_rad = dt;
            _last_dt = dt;
            return dt;
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
        /**
         * @brief Get quantization error of measurement
         * 
         * @return double The quantization error of measurement
        */
        double getQuantizationError() {
            return 1. / (double) _ppr * (double) _gear_ratio / (double) _last_dt * 1e6 * 2. * M_PI;
        }
    private:
        long _last_update_time = 0;
        double _radps = 0;
        double _cumulative_rad = 0;
        double _last_dt = 0;
        // Params
        int _hall_a_pin;
        int _hall_b_pin;
        int _ppr;
        double _gear_ratio;
        long _target_dt;
        // ISR vars
        volatile long total_count = 0;
        volatile long delta_count = 0;
        volatile bool dirty = false;
};