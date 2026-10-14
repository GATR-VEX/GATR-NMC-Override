#pragma once

#include "globals.hpp"

namespace intake {
    enum class WantedState {
        CLOSED,
        OPEN
    };

    enum class SystemState {
        CLOSED_LOADED,
        CLOSED_UNLOADED,
        OPEN
    };

    void initialize();
    void periodic();

    void setWantedState(WantedState state);

    WantedState getWantedState();
    SystemState getSystemState();
}