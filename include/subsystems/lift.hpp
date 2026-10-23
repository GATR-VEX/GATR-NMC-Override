#pragma once

#include "helpers.hpp"
#include "globals.hpp"
#include "constants.hpp"
#include "utilities/motion_profiler.hpp"
#include "utilities/arm_feedforward.hpp"
#include "utilities/pid.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace lift {
    enum class WantedState {
        IDLE,
        HOME,
        MOVE_TO_POSITION,
    };

    enum class SystemState {
        IDLING,
        HOMING,
        MOVING_TO_POSITION,
    };

    void initialize();
    void periodic();

    void set_wanted_state(WantedState state);

    WantedState get_wanted_state();
    SystemState get_system_state();

    void set_target_height(double height_inches);
    double get_current_height();

    double get_position();
    double get_velocity();

    bool is_homed();
    bool at_setpoint();

    void screen_subtask();
}
