#pragma once

#include <cstdint>

namespace constants {
    inline constexpr double PI =  3.1415926;
    inline constexpr bool TUNING_MODE = true;
    inline constexpr double DT_SECONDS = 0.1;

    namespace drivetrain {

    }

    namespace lift {
        inline constexpr double KP = 50.0;
        inline constexpr double KD = 0.0;
        inline constexpr double KS = 1.0;
        inline constexpr double KV = 0.0;
        inline constexpr double KG = 0.5;
        inline constexpr double KA = 0.0;

        inline constexpr double ZERO_VELOCITY_THRESHOLD = 0.125;
        inline constexpr uint32_t HOMING_SPIN_UP_MS = 100;
        inline constexpr uint32_t ZERO_VELOCITY_TIME_MS = 300;
        inline constexpr uint32_t HOMING_TIMEOUT_MS = 3000;
        inline constexpr int HOMING_VOLTAGE = -12;

        inline constexpr double MECHANISM_HOMING_ANGLE = -0.575959;
        inline constexpr double MECHANISM_HOMING_POSITION = 2.0;

        inline constexpr double MECHANISM_REDUCTION = (72.0 / 24.0);
        inline constexpr double MINIMUM_POSITION_INCHES = 0.0;
        inline constexpr double MAXIMUM_POSITION_INCHES = 47.0;
        inline constexpr double MAXIMUM_ANGLE_RADIANS = (PI / 4.0);

        inline constexpr double POSITION_TOLERENCE_INCHES = 1.0;
        inline constexpr double VELOCITY_TOLERENCE_INCHES_PER_SECOND = 3.0;
        inline constexpr double MAXIMUM_VELOCITY_RADIANS_PER_SECOND = 4.0;
        inline constexpr double MAXIMUM_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 12.0;

        inline constexpr double STAGE_LENGTH_INCHES = 12.0;
        inline constexpr int STAGE_COUNT = 2;

        // Only used for testing and profiling lift:
        inline constexpr double VOLTAGE_COARSE_STEP = 0.10;
        inline constexpr double VOLTAGE_FINE_STEP = 0.02;
        inline constexpr double PROFILING_VOLTAGE_LIMIT = 6.0;

        inline constexpr double TOGGLE_HEIGHT = 12.0;
    }

    namespace intake {

    }

    namespace superstructure {
        inline constexpr double ALLIANCE_BASE_HEIGHT_IN = 0.0;
        inline constexpr double SHORT_BASE_HEIGHT_IN = 0.0;
        inline constexpr double TALL_BASE_HEIGHT_IN = 0.0;

        inline constexpr double STACK_INCREMENT_IN = 4.0;
        inline constexpr int MAX_STACK_COUNT = 5;
    }
}