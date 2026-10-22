#pragma once

#include <cstdint>

namespace constants {
    inline constexpr double kPi =  3.1415926;
    inline constexpr bool kTuningMode = true;

    namespace drivetrain {

    }

    namespace lift {
        inline constexpr double kP = 0.0;
        inline constexpr double kD = 0.0;
        inline constexpr double kS = 0.0;
        inline constexpr double kV = 0.0;
        inline constexpr double kG = 9000.0;
        inline constexpr double kA = 0.0;

        inline constexpr double kZeroVelocityThreshold = 0.125;

        inline constexpr uint32_t kZeroVelocityTimeMs = 300;
        inline constexpr uint32_t kHomingTimeoutMs = 5000;
        inline constexpr int kHomingPower = -12000;

        inline constexpr double kMotorReduction = 1.0;
        inline constexpr double kMinimumPosition = 0.0;
        inline constexpr double kMaxiumumPosition = 47.0;

        inline constexpr double kPositionTolerence = 2.0;
        inline constexpr double kVelocityTholerence = 2.0;

        inline constexpr double kArmLengthInches = 16.5;
    }

    namespace intake {

    }
}