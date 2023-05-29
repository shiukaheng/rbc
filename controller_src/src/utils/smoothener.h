#pragma once
#include <Arduino.h>

class Smoothener {
    public:
        Smoothener(int size);
        ~Smoothener();
        double update(double value);
    private:
        double* _buffer;
        int _buffer_size;
        int _buffer_index;
        double _buffer_sum;
};

// TODO: Time-based smootheners!

// /**
//  * @brief Variant of smoothener that uses a time interval instead of just a buffer size
//  * 
//  * With a time interval, the number of samples could be infinite, but the buffer size is fixed (for memory and performance reasons)
//  * So, we implement a novel algorithm:
//  * - We want to insert a new value into the buffer and get the average
//  *   - We free all outdated values (outside the time interval)
//  *   - Is the buffer still full?
//  *       - No: We insert the new value and update the sum
//  *       - Yes: We randomly pick an index between 0 and the number of valid values (which will have 1 extra),
//  *              if the range is in 0 - (num valid values - 1), we replace the value at that index with the new value,
//  *              or if the value is at the end of the range, we ignore the new value
//  * 
//  * Why this works:
//  * - If the buffer is big enough to store all values given in the time interval, then the buffer will never be full,
//  *   giving us the exact average we want,
//  * - When the buffer is small, it is equivalent to a random sample of the values in the time interval, which will be
//  *   a good approximation of the average, and improves as the buffer size increases
//  */
// class TimeWindowSmoothener {
//     public:
//         TimeSmoothener(int milliseconds, int buffer_size);
//         ~TimeSmoothener();
//         double update(double value);
//     private:
//         double* _time_buffer;
//         double* _value_buffer;
//         bool* _valid;
// };

// // Alternative implementation with no value approximation but has less time resolution:
// // If the buffer is full, we downsample the whole buffer by 2, and insert the new value at the end
// // Advantage: No approximation, no random sampling
// // Disadvantage: Less time resolution, more memory usage

// // Yet another alternative for similar but not the same purpose:

class ExpTimeSmoothener {
    public:
        ExpTimeSmoothener(double decay);
        double update(double value);
    private:
        double _last_value;
        unsigned long _last_time;
        double _decay;
        bool _first = true;
};