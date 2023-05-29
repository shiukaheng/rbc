#pragma once

#include <Arduino.h>

// TODO: Transition towards making all classes' update functions to use delta time given by the clock class
//       to prevent overflow issues, and save on memory and performance

struct Tick
{
    // Contains time now and time delta
    unsigned long now;
    unsigned long delta;
};


/**
 * @brief A class that keeps track of time by providing delta nanoseconds since the last update
 * 
 */
class Clock {
    private:
        unsigned long _lastUpdate;
    public:
        Clock();
        Tick update();
};