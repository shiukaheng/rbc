#pragma once
#include "time.h"

template <typename T>
class BaseStateUpdater {
    public:
        T& const state;
        BaseStateUpdater(T& state) : state(state) {}
        void update(Tick& tick) {
        }
};