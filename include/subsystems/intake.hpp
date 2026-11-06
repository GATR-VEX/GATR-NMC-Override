#pragma once

#include "globals.hpp"
#include "constants.hpp"

namespace intake {
    enum class WantedState {
        CLOSED,
        OPEN,
	AUTO_INTAKE
    };

    enum class SystemState {
        CLOSED_LOADED,
        CLOSED_UNLOADED,
        OPENED,
	WAITING_FOR_PIECE
    };

    void initialize();
    void periodic();

    void set_wanted_state(WantedState state);

    WantedState get_wanted_state();
    SystemState get_system_state();

    bool has_game_piece();
    void screen_subtask();
}
