#include "smoothener.h"

Smoothener::Smoothener(int size) {
    _buffer_size = size;
    _buffer = new double[size]; // Allocate the buffer
    memset(_buffer, 0, sizeof(double) * size); // Initialize the buffer to 0
    _buffer_index = 0;
    _buffer_sum = 0;
}

Smoothener::~Smoothener() {
    delete[] _buffer;
}

double Smoothener::update(double value) {
    _buffer_sum -= _buffer[_buffer_index]; // Remove the oldest value from the sum
    _buffer[_buffer_index] = value; // Add the new value to the buffer
    _buffer_sum += value; // Add the new value to the sum
    _buffer_index = (_buffer_index + 1) % _buffer_size; 
    return _buffer_sum / _buffer_size;
}

ExpTimeSmoothener::ExpTimeSmoothener(double decay) {
    _decay = decay;
    _last_value = 0;
    _last_time = 0;
}

double ExpTimeSmoothener::update(double value) {
    unsigned long now = micros();
    double delta_seconds = (now - _last_time) / 1000000.0;
    _last_time = now;
    double decay = pow(_decay, delta_seconds); // Calculate the decay factor, such that the value decays by _decay in 1 second
    if (_first) {
        _first = false;
        _last_value = value;
    } else {
        _last_value = _last_value * decay + value * (1 - decay); // Apply the decay factor
    }
    return _last_value;
}