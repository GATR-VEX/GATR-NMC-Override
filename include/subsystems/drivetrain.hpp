#pragma once

#include "helpers.hpp"
#include "globals.hpp"

namespace drivetrain {
    enum class Behavior {
        TANK,
        SPLIT_ARCADE,
        SINGLE_ARCADE
    };

    enum class WantedState {
        TELEOP_DRIVE,
        DRIVE_TO_POINT,
        IDLE
    };

    enum class SystemState {
        TELEOP_DRIVE,
        DTP_ALIGNING,
        DTP_ALIGNED,
        IDLING
    };

    void initialize();
    void prepare_teleop();
    void prepare_autonomous();
    void periodic();

    void set_wanted_state(WantedState state);

    WantedState get_wanted_state();
    SystemState get_system_state();

    void set_behavior(Behavior behavior);
    Behavior get_behavior();

    bool is_imu_calibrating();
    bool is_imu_calibrated();

    void set_target_position(double x, double y);

    void update_pid();
    void screen_subtask();
}
