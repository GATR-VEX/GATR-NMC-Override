#pragma once

#include "helpers.hpp"
#include "globals.hpp"
#include "constants.hpp"
#include "utilities/feedforward.hpp"
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

    void setWantedState(WantedState state);

    WantedState getWantedState();
    WantedState getWantedState(double targetPosition);
    SystemState getSystemState();

    void setTargetHeight(double heightInches);
    double getCurrentHeight();

    double getPosition();
    double getVelocity();

    void setTargetPosition();

    bool isHomed();
    bool atSetpoint();
}