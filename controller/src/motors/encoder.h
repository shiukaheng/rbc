#pragma once

#include <Arduino.h>

#define PPR 16
#define GEAR_RATIO 1./82.
#define MIN_DT 50000 // Minimum time for an update to collect pulses

// Potential bug: _dirty not handled properly
// TODO: Add regular interrupt pin as a parameter
// TODO: Rename methods for standardization (i.e., isr, update, getRPS)

/* USAGE:
 * 1. Create an EncoderReader object:
 *      EncoderReader* encoder = new EncoderReader(hall_a_pin, hall_b_pin);
 * 2. Create a static ISR function that calls the isr() function of the encoder object:
 *      void isr() {
 *         encoder->isr();
 *      }
 * 3. Attach the ISR to the interrupt pin:
 *      attachInterrupt(digitalPinToInterrupt(INT_PIN), isr, RISING);
 * 4. Call update() in the main loop and get the rad/s:
 *       void loop() {
 *          ...
 *          encoder->update();
 *          double rps = encoder->getRPS();
 *          ...
 *       }
 */

// ========== MAIN CLASS DECLARATION =========

/**
 * @brief A class that monitors the encoder and calculates the rad/s
 * 
 */
class EncoderReader {
    private:
        // Params
        int _hall_a_pin;
        int _hall_b_pin;
        int _ppr;
        double _gear_ratio;
        long _min_dt;
        // ISR vars
        volatile int _current_pulses = 0;
        volatile int _pulse_time_history_index = -1;
        // volatile long _pulse_time_history[PPR+1]; // So we will have PPR pulse durations to work with
        volatile long* _pulse_time_history;
        // volatile bool _pulse_cws[PPR+1];
        volatile bool* _pulse_cws;
        volatile bool _dirty = true;
        volatile int _total_pulses = 0;
        volatile bool _clear_period_vars = false;
        // volatile bool cws = true;
        // Update vars
        long _last_update_time = 0;
        bool _first_update = true;
        double _radps = 0;
        bool _period_method = false;
    public:
        EncoderReader(int hall_a_pin, int hall_b_bin, int ppr = PPR, double gear_ratio = GEAR_RATIO, long min_dt = MIN_DT);
        ~EncoderReader();
        /**
         * @brief The ISR hook that should be triggered by the A pin of the encoder
         * 
         */
        void isr();
        /**
         * @brief Updates the monitor
         * 
         */
        void update();
        /**
         * @brief Gets the rad/s
         * 
         * @return double The rad/s
         */
        double getRPS();
};