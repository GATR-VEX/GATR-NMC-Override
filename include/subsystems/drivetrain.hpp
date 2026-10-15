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
        IDLE
    };

    void initialize();
    void prepareTeleop();
    void prepareAutonomous();
    void periodic();

    void setWantedState(WantedState state);

    WantedState getWantedState();
    SystemState getSystemState();

    void setBehavior(Behavior behavior);
    Behavior getBehavior();

    bool isIMUCalibrating();
    bool isIMUCalibrated();

    void setTargetPosition(double x, double y);

    void updatePID();
    void printDebugScreen();
}