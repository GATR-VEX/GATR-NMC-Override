#pragma once

#include "EZ-Template/api.hpp"
#include "globals.hpp"

namespace drivetrain {
    enum class Behavior {
        TANK,
        SPLIT_ARCADE,
        SINGLE_ARCADE,
        FLIPPED_SPLIT_ARCADE,
        FLIPPED_SINGLE_ARCADE
    };

    void initialize();

    void setBehavior(Behavior behavior);
    Behavior getBehavior();

    void periodic();
}