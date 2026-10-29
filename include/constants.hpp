#pragma once

#include <cstdint>

namespace constants {
    inline constexpr double PI =  3.1415926;
    inline constexpr bool TUNING_MODE = true;

    namespace drivetrain {

    }

    namespace lift {
        inline constexpr double KP = 0.0;
        inline constexpr double KD = 0.0;
        inline constexpr double KS = 1.990;
        inline constexpr double KV = 0.0;
        inline constexpr double KG = 1.810;
        inline constexpr double KA = 0.0;

        inline constexpr double ZERO_VELOCITY_THRESHOLD = 0.125;

        inline constexpr uint32_t ZERO_VELOCITY_TIME_MS = 300;
        inline constexpr uint32_t HOMING_TIMEOUT_MS = 5000;
        inline constexpr int HOMING_VOLTAGE = -12000;
        inline constexpr double MECHANISM_HOMING_ANGLE = (-PI) / 4.0;

        inline constexpr double MECHANISM_REDUCTION = (48.0 / 16.0);
        inline constexpr double MINIMUM_POSITION_INCHES = 0.0;
        inline constexpr double MAXIMUM_POSITION_INCHES = 47.0;

        inline constexpr double POSITION_TOLERENCE_INCHES = 2.0;
        inline constexpr double VELOCITY_TOLERENCE_INCHES_PER_SECOND = 2.0;
        inline constexpr double MAXIMUM_VELOCITY_INCHES_PER_SECOND = 30.0;
        inline constexpr double MAXIMUM_ACCELERATION_INCHES_PER_SECOND_SQUARED = 60.0;

        inline constexpr double ARM_LENGTH_DR4B = 28.7;
    }

    namespace intake {

    }
}