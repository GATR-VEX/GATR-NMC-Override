#pragma once

#include <cstdint>
#include <array>

namespace constants {
    inline constexpr double PI =  3.1415926;
    inline constexpr bool TUNING_MODE = true;

	inline constexpr double OPCONTROL_LENGTH_SECONDS = 105;
    inline constexpr double DT_SECONDS = 0.1;
    inline constexpr double MAXIMUM_LOOP_DT_SECONDS = 0.1;

    namespace drivetrain {
        inline constexpr double TIP_PITCH_DEGREES = 24.0;
        inline constexpr uint32_t TIP_CONFIRM_MS = 100;
    }

    namespace lift {
        inline constexpr double KP = 120.0;
        inline constexpr double KD = 0.0;
        inline constexpr double KS = 1.7;
        inline constexpr double KV = 2.0;
        inline constexpr double KG = 0.75;
        inline constexpr double KA = 0.01;

        inline constexpr std::array<double, 9> GRAVITY_REGRESSION_ANGLE_DEGREES = {-30.0, -15.0, 0.0, 15.0, 30.0, 45.0, 60.0, 75.0, 90.0};
        inline constexpr std::array<double, 9> GRAVITY_REGRESSION_VOLTAGE = {1.9, 3.2, 3.5, 3.7, 4.3, 4.4, 4.5, 4.7, 5.0};

        inline constexpr double ZERO_VELOCITY_THRESHOLD = 0.125;
        inline constexpr uint32_t HOMING_SPIN_UP_MS = 200;
        inline constexpr uint32_t ZERO_VELOCITY_TIME_MS = 300;

        inline constexpr uint32_t HOMING_SETTLE_MS = 300;
        inline constexpr uint32_t HOMING_TIMEOUT_MS = 3000;
        inline constexpr int HOMING_VOLTAGE = -9;

        inline constexpr double BOTTOM_HOLD_VOLTAGE = 0.0;
        inline constexpr double BOTTOM_HOLD_BAND_RADIANS = 1.5 * PI / 180.0;

        inline constexpr double MECHANISM_HOMING_ANGLE = -(PI / 6.0);
        inline constexpr double MECHANISM_HOMING_POSITION = 2.625;

        inline constexpr double MECHANISM_REDUCTION = (72.0 / 24.0);
        inline constexpr double MINIMUM_POSITION_INCHES = 0.0;
        inline constexpr double MAXIMUM_POSITION_INCHES = 47.0;
        inline constexpr double MAXIMUM_ANGLE_RADIANS = (PI / 2.0);

        inline constexpr double POSITION_TOLERENCE_INCHES = 1.0;
        inline constexpr double VELOCITY_TOLERENCE_INCHES_PER_SECOND = 3.0;
        inline constexpr double MAXIMUM_VELOCITY_RADIANS_PER_SECOND = 2.6;
        inline constexpr double MAXIMUM_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 4.0;

        inline constexpr double STAGE_LENGTH_INCHES = 12.125;
        inline constexpr int STAGE_COUNT = 2;

        inline constexpr double LIFT_DOWN_MARGIN_IN = 3.0;
    }

    namespace intake {
		inline constexpr int GAME_PIECE_DETECTION_DISTANCE_MM = 80;
		inline constexpr int GAME_PIECE_CLEAR_DISTANCE_MM = 100;
		inline constexpr uint32_t PIECE_DETECTION_TIME_MS = 60;
    }

    namespace superstructure {
        inline constexpr std::size_t STACK_POSITIONS = 10;

		inline constexpr std::array<double, STACK_POSITIONS> ALLIANCE_NODE_HEIGHTS = {17.5, 21.125, 24.0, 28.0, 30.125, 33.0, 35.0, 36.875, 38.25, 39.5};
		inline constexpr std::array<double, STACK_POSITIONS> NEUTRAL_NODE_HEIGHTS = {20.875, 25.0, 27.875, 30.5, 32.5, 35.0, 36.875, 38.25, 39.0, 45.0};
		inline constexpr std::array<double, STACK_POSITIONS> CENTER_NODE_HEIGHTS = {24.25, 27.5, 30.0, 32.75, 34.25, 36.5, 38.0, 40.25, 40.25, 40.25};
    }
}
