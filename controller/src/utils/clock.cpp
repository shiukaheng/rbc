#include "clock.h"

Clock::Clock() {
    _lastUpdate = micros();
}

Tick Clock::update() {
    unsigned long now = micros();
    unsigned long delta = now - _lastUpdate;
    // TODO: If delta is overflowed between now and last update, we need to handle it here
    _lastUpdate = now;
    return Tick{now, delta};
}