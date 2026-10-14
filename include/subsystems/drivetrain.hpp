#pragma once

#include "helper.h"
#include "globals.hpp"

namespace drivetrain {
    enum class Behavior {
        TANK,
        SPLIT_ARCADE,
        SINGLE_ARCADE
    };

    enum class WantedState {
        TELEOP_DRIVE,
        HEADING_LOCK,
        DRIVE_TO_POINT,
        IDLE
    };

    enum class SystemState {
        TELEOP_DRIVE,
        HOLDING_HEADING,
        DTP_ALIGNING,
        DTP_ALIGNED,
        IDLE
    };

    void initialize();
    void periodic();

    void setWantedState(WantedState state);

    WantedState getWantedState();
    SystemState getSystemState();

    void setBehavior(Behavior behavior);
    Behavior getBehavior();

    bool isIMUCalibrating();
    bool isIMUCalibrated();

    void setTargetHeading(double degrees);
    void setTargetPosition(double x, double y);
}