#pragma once

#include <Arduino.h>

#define PPR 16
#define GEAR_RATIO 1./82.

class EncoderReaderNaive{
    public:
        EncoderReaderNaive(int hall_a_pin, int hall_b_bin, int ppr = PPR, double gear_ratio = GEAR_RATIO, long target_dt = 30000, double max_abs_acceleration = 200.) {
            _hall_a_pin = hall_a_pin;
            _hall_b_pin = hall_b_bin;
            _ppr = ppr;
            _gear_ratio = gear_ratio;
            _target_dt = target_dt;
            _max_abs_acceleration = max_abs_acceleration;
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
            if (clear_delta_count) {
                delta_count = 0;
                clear_delta_count = false;
            }
            if (cw) {
                delta_count++;
            } else {
                delta_count--;
            }
        }
        /**
         * @brief Updates the monitor
         * 
         */
        long update() {

            // STAGE 1: Read the data from the ISR

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
            long delta_count_copy = delta_count;
            if (dirty) { // If dirty, means that the ISR was triggered during the update, so we re-read the data in the next update
                return -1;
            }

            // STAGE 2: Process the data

            // Up till this point, we know we have read data that was not corrupted by the ISR, but we don't know if there were noisy pulses
            // that misfires the ISR. We will check for that now by thresholding a maximally possible acceleration, and not updating the
            // state if the acceleration is too high.

            // Reset the delta count, so we can begin counting again
            clear_delta_count = true;

            // Update the last update time
            _last_update_time = now;

            // Calculate the rad/s
            double radps = (double) delta_count_copy / (double) _ppr * (double) _gear_ratio / (double) dt * 1e6 * 2. * M_PI;

            // Calculate the acceleration (absoulte value)
            double radps2 = abs(radps - _last_radps) / (double) dt * 1e6;

            _last_radps = _radps; // Used for calculating acceleration

            // If the acceleration is too high, return
            if (radps2 > _max_abs_acceleration) {
                return -1;
            }

            // STAGE 3: Update the state

            // If we arrive at this point, it means that the data is valid, and we can update the state

            _radps = radps; // _radps gets read by the getter, radps is just the draft value after we confirm it wasnt a false reading
            _cumulative_rad = (double) total_count / (double) _ppr * (double) _gear_ratio * 2. * M_PI;
            _last_dt = dt; // Used for calculating quantization error
            
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
        double _last_radps = 0;
        double _cumulative_rad = 0;
        double _last_dt = 0;
        // Params
        int _hall_a_pin;
        int _hall_b_pin;
        int _ppr;
        double _gear_ratio;
        long _target_dt;
        double _max_abs_acceleration = 0;
        // ISR vars
        volatile long total_count = 0;
        volatile long delta_count = 0;
        volatile bool dirty = false;
        volatile bool clear_delta_count = false;
};