#pragma once

#include <cstdint>
#include <array>

namespace constants {
    inline constexpr double PI =  3.1415926;
    inline constexpr bool TUNING_MODE = true;
    inline constexpr double DT_SECONDS = 0.1;
    inline constexpr double MAXIMUM_LOOP_DT_SECONDS = 0.1;

    namespace drivetrain {

    }

    namespace lift {
        inline constexpr double KP = 150.0;
        inline constexpr double KD = 0.0;
        inline constexpr double KS = 1.5;
        inline constexpr double KV = 0.5;
        inline constexpr double KG = 0.75;
        inline constexpr double KA = 0.1;

        inline constexpr std::array<double, 8> GRAVITY_REGRESSION_ANGLE_DEGREES = {-30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0};
        inline constexpr std::array<double, 8> GRAVITY_REGRESSION_VOLTAGE = {1.8, 2.8, 3.15, 3.3, 4.2, 4.3, 4.4, 4.5};

        inline constexpr double ZERO_VELOCITY_THRESHOLD = 0.125;
        inline constexpr uint32_t HOMING_SPIN_UP_MS = 200;
        inline constexpr uint32_t ZERO_VELOCITY_TIME_MS = 300;

        inline constexpr uint32_t HOMING_SETTLE_MS = 300;
        inline constexpr uint32_t HOMING_TIMEOUT_MS = 3000;
        inline constexpr int HOMING_VOLTAGE = -12;

        inline constexpr double BOTTOM_HOLD_VOLTAGE = -2.5;
        inline constexpr double BOTTOM_HOLD_BAND_RADIANS = 1.5 * PI / 180.0;

        inline constexpr double MECHANISM_HOMING_ANGLE = -(PI / 6.0);
        inline constexpr double MECHANISM_HOMING_POSITION = 2.625;

        inline constexpr double MECHANISM_REDUCTION = (72.0 / 24.0);
        inline constexpr double MINIMUM_POSITION_INCHES = 0.0;
        inline constexpr double MAXIMUM_POSITION_INCHES = 47.0;
        inline constexpr double MAXIMUM_ANGLE_RADIANS = (PI / 4.0);

        inline constexpr double POSITION_TOLERENCE_INCHES = 1.0;
        inline constexpr double VELOCITY_TOLERENCE_INCHES_PER_SECOND = 3.0;
        inline constexpr double MAXIMUM_VELOCITY_RADIANS_PER_SECOND = 2.3;
        inline constexpr double MAXIMUM_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 5.0;

        inline constexpr double STAGE_LENGTH_INCHES = 12.125;
        inline constexpr int STAGE_COUNT = 2;

        inline constexpr double TOGGLE_HEIGHT = 12.0;
    }

    namespace intake {
		inline constexpr int GAME_PIECE_DETECTION_DISTANCE_MM = 80;
		inline constexpr int GAME_PIECE_CLEAR_DISTANCE_MM = 100;
		inline constexpr uint32_t PIECE_DETECTION_TIME_MS = 60;
    }

    namespace superstructure {
        inline constexpr double ALLIANCE_BASE_HEIGHT_IN = 0.0;
        inline constexpr double SHORT_BASE_HEIGHT_IN = 0.0;
        inline constexpr double TALL_BASE_HEIGHT_IN = 0.0;

        inline constexpr double PIN_OFFSET_IN = 0.0;
        inline constexpr double CUP_OFFSET_IN = 0.0;
        inline constexpr double BOTH_OFFSET_IN = 0.0;

        inline constexpr double STACK_INCREMENT_IN = 4.0;
        inline constexpr int MAX_STACK_COUNT = 5;
    }
}
