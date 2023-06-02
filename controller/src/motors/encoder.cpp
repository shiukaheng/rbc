#include "encoder.h"

// ========== CONVENIENCE FUNCTIONS ==========:
/**
 * @brief Remaps the current index by the offset
 * 
 * @param current_index The newest index in the array
 * @param offset The offset (reading back in time by n offsets), maximum would be max_index
 * @param max_index The maximum index of the array
 * @return int The remapped index
 */
int remap_index(int current_index, int offset, int max_index) {
    return (current_index - offset + max_index) % max_index;
}

/**
 * @brief Reads the dt from the pulse time history
 * 
 * @param current_index The newest index in the array
 * @param offset The offset (reading back in time by n offsets), maximum would be max_index - 1 (since for dt, we need to read the current index and the index before it)
 * @param max_index The maximum index of the array
 * @param pulse_time_history The pulse time history array
 * @return long The dt
 */
int read_dt(int current_index, int offset, int max_index, long pulse_time_history[], bool pulse_cws[]) {
    // Read direction
    bool cws = pulse_cws[remap_index(current_index, offset, max_index)]; // Will be true if the wheel is spinning clockwise, false if counter-clockwise
    int sign = cws ? 1 : -1;
    // Serial.println(sign);
    int unsigned_dt = pulse_time_history[remap_index(current_index, offset, max_index)] - pulse_time_history[remap_index(current_index, offset + 1, max_index)];
    return unsigned_dt * sign;
}

/**
 * @brief Converts the pulse period to wheel rad/s
 * 
 * @param period The pulse period
 * @return double The wheel rad/s
 */
double period_to_rads_per_sec(int period, int ppr, double gear_ratio) {
    // Microseconds in a second / period / pulses per revolution / gear down ratio
    return (2 * M_PI) / (period / 1e6) / ppr * gear_ratio;
}

// ========== IMPLEMENTATION ==========

EncoderReader::EncoderReader(int hall_a_pin, int hall_b_bin, int ppr = PPR, double gear_ratio = GEAR_RATIO, long min_dt = MIN_DT) {
    _hall_a_pin = hall_a_pin;
    _hall_b_pin = hall_b_bin;
    _ppr = ppr;
    _gear_ratio = gear_ratio;
    _min_dt = min_dt;
    // Allocate memory for the arrays
    _pulse_time_history = new long[ppr + 1];
    _pulse_cws = new bool[ppr + 1];
    pinMode(_hall_b_pin, INPUT);
}

EncoderReader::~EncoderReader() {
    delete[] _pulse_time_history;
    delete[] _pulse_cws;
}

void EncoderReader::isr() {
    if (_clear_period_vars) {
        _pulse_time_history_index = -1;
        _total_pulses = 0;
        _clear_period_vars = false;
        // _pulse_time_history and _pulse_cws don't have to be cleared, since they will be overwritten
    }
    // Increment the index
    _pulse_time_history_index++;
    // Check if the index is out of bounds
    if (_pulse_time_history_index >= _ppr) {
        _pulse_time_history_index = 0;
    }
    // Write the time to the history
    _pulse_time_history[_pulse_time_history_index] = micros();
    // Write the direction to the history
    bool cws = digitalRead(_hall_b_pin);
    _pulse_cws[_pulse_time_history_index] = cws;
    // Increment the number of pulses
    if (cws) {
        _current_pulses++;
        // Serial.println("CWS");
    } else {
        _current_pulses--;
        // Serial.println("CCWS");
    }
    if (_total_pulses < _ppr + 1) { // Capped to prevent overflow
        _total_pulses++;
    }
    _dirty = true;
}

// Update
void EncoderReader::update() {

    // ===== Initialize update by grabbing the right variables =====

    // Get the current time
    long current_time = micros();

    // Copy the pulse time history, instead of working directly with the volatile array, which may get modified mid-calculation
    _dirty = false; // Dirty flag should stay false during the copying process, or else we restart the calculation

    int index = _pulse_time_history_index;
    long pulse_time_history[_ppr+1];
    bool pulse_cws[_ppr+1];
    int total_pulses = _total_pulses;
    for (int i = 0; i < _ppr; i++) {
        pulse_time_history[i] = _pulse_time_history[i];
        pulse_cws[i] = _pulse_cws[i];
    }

    // Compute the latest pulse time duration, which will be used for:
    // i. Predicting the next pulse time to choose which method to use (that's why initially its only loaded with one value)
    // ii. Computing the average pulse time duration (same variable will be recycled to save memory, by recursively updating it using weighted average)
    double first_pulse_period = read_dt(index, 0, _ppr, pulse_time_history, pulse_cws);
    double avg_radps = period_to_rads_per_sec(first_pulse_period, _ppr, _gear_ratio);
    int num_added = 1;

    long last_pulse_time = pulse_time_history[index]; // For convenience

    /*
    Original thought process (Version A):

    We know the last pulse time and its duration. Predictively, the next pulse time should be around:
    last pulse time + last pulse period (assuming the period is constant)

    If this theorectical next pulse time is in the future (after this update), we assume the motion is still happening, probably.

    However, if the predicted pulse should have already happened, but it hasn't (before this update), 
    we assume the motion has stopped or at least significantly slowed down, and we fall back to the average pulse time duration,
    which is more robust to slow speeds.

    Obviously, there is noise, and there is also the possibility that the motion is actually slowing down, and that the next pulse time
    is just slightly delayed. In this case, we could inflate the the average pulse time duration by a factor of 2, and use that as the
    next pulse time. (Or alternatively, with a constant added to the average pulse time duration).

    If we interpret this with some basic physics, we are simply falling back when the deceleration reaches a certain threshold. (I think)
    TODO: It would probably be wise to interpret those constants as physical parameters (i.e., max decceleration)
    
    Conflicting thoughts (VERSION B):

    Using the current system, when the system deccelerates, from a very fast speed to a fast speed, it would still trigger the fallback method,
    which yields a very inaccurate speed (a window can only capture so many pulses), perhaps we should use a hard threshold to wait for the
    next pulse to happen, and if it doesn't, we fall back to the average pulse time duration.

    This could be interpreted as using the fallback method when the velocity is below a certain threshold, which may yield better results if
    most of the time the velocity is above the threshold.
    */

    // If dirty, we restart the calculation
    if (_dirty) {
        return;
    }

    // long next_pulse_time = last_pulse_time + first_pulse_period * 5; // Version A
    long next_pulse_time = last_pulse_time + _min_dt * 2; // Version B. 2 is a magic number, but it seems to work well
    
    // Checking conditions to see if we should use the period-based method
    bool in_future = (next_pulse_time - current_time)>=0;
    // Print the earliest pulse time in the window
    // Serial.println(pulse_time_history[(index + 1) % PPR] - current_time);
    bool use_period_method = (total_pulses >= (_ppr + 1)) && in_future;

    // Create string that shows why we are not using the period method
    // if (total_pulses < (_ppr + 1)) {
    //     Serial.println("Not enough pulses: " + String(total_pulses));
    // } else if (!in_future) {
    //     Serial.println("Next pulse time is in the past");
    // }

    // Period-based speed calculation; only update _radps if the conditions are met
    if (use_period_method) {
        // Serial.println("This is the period method");
        _period_method = true;
        for (int i = 1; i < _ppr-1; i++) { // HACK! : Have not throughouly thought out why its i < PPR-1, but it works for now.
            long pulse_dt = read_dt(index, i, _ppr, pulse_time_history, pulse_cws);
            // Serial.println(pulse_dt);
            // Update the average
            avg_radps = (avg_radps * num_added + period_to_rads_per_sec(pulse_dt, _ppr, _gear_ratio)) / (num_added + 1);
            num_added++;
        }
        // Calculate the rad/s
        _radps = avg_radps;
    } else {
        // When we switch from period-based method to window-based method, we need to reset the accumulator to clear irrelevant pulses
        // Serial.println("This is the window method");
        if (_period_method) {
            _clear_period_vars = true;
        }
        _period_method = false;
    }

    // Window-based speed calculation
    // We use a integer that is incremented every time the ISR is called
    // Combined with the current time and when the the accumulator was last cleared, we can calculate the speed
    // (i.e., speed = pulses / dt / PPR * 60)
    long dt = current_time - _last_update_time;
    if (dt >= MIN_DT) { // We don't want to update too often, as having a tiny window will yield a very inaccurate speed, so we set a minimum dt
        // Calculate the rad/s
        double window_radps = _current_pulses / (dt / 1e6) / _ppr * _gear_ratio * 60;
        // Reset the number of pulses
        _current_pulses = 0;
        // Update the last update time
        _last_update_time = current_time;
        // Update the rad/s
        if (!use_period_method) {
            _radps = window_radps;
        }
    }
}

double EncoderReader::getRPS() {
    return _radps;
}
