#pragma once

#include "globals.hpp"
#include "constants.hpp"

namespace intake {
    enum class WantedState {
        CLOSED,
        OPEN
    };

    enum class SystemState {
        CLOSED_LOADED,
        CLOSED_UNLOADED,
        OPENED
    };

    void initialize();
    void periodic();

    void set_wanted_state(WantedState state);

    WantedState get_wanted_state();
    SystemState get_system_state();

    void screen_subtask();
}
