#pragma once

#include "globals.hpp"
#include "constants.hpp"
#include "subsystems/lift_kinematics.hpp"

#include <cstdint>

namespace lift {
    enum class WantedState {
        IDLE,
        HOME,
        MOVE_TO_POSITION,
        PROFILE, // Only used to profile the lift during test sessions
    };

    enum class SystemState {
        IDLING,
        HOMING,
        MOVING_TO_POSITION,
        PROFILING, // Only used to profile the lift during test sessions
    };

    void initialize();
    void periodic();

    void set_wanted_state(WantedState state);

    WantedState get_wanted_state();
    SystemState get_system_state();

    void set_target_height(double height_inches);
    double get_target_height();
    double get_current_height();

    double get_angle();
    double get_angular_velocity();
    double get_linear_velocity();

    bool is_homed();
    bool at_setpoint();

    void adjust_profiling_voltage(double delta_volts);
    double get_profiling_voltage();

    void screen_subtask();
}