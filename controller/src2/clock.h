#pragma once
#include <Arduino.h>

struct Tick {
    double dt; // 
    long time; // Time in microseconds
};

class Clock {
    private:
        long last_update_time;
    public:
        Clock() {
            last_update_time = 0;
        }
        Tick update() {
            long current_time = micros();
            double dt = (double) (current_time - last_update_time) / 1000000.;
            last_update_time = current_time;
            return { dt, current_time };
        }
};