#include "subsystems/lift_kinematics.hpp"
#include "constants.hpp"

#include <algorithm>
#include <cmath>

namespace lift::kinematics {
    namespace {
        constexpr double EFFECTIVE_LENGTH_INCHES = constants::lift::STAGE_COUNT * constants::lift::STAGE_LENGTH_INCHES;
    }

    double motor_rotations_to_angle(double motor_rotations) {
        const double bar_rotations = motor_rotations / constants::lift::MECHANISM_REDUCTION;
        return constants::lift::MECHANISM_HOMING_ANGLE + bar_rotations * 2.0 * constants::PI;
    }

    double motor_rpm_to_angular_velocity(double motor_rpm) {
        const double bar_rotations_per_second = (motor_rpm / 60.0) / constants::lift::MECHANISM_REDUCTION;
        return bar_rotations_per_second * 2.0 * constants::PI;
    }

    double angle_to_height(double angle_radians) {
        return constants::lift::MECHANISM_HOMING_POSITION + constants::lift::STAGE_COUNT * constants::lift::STAGE_LENGTH_INCHES * (std::sin(angle_radians) - std::sin(constants::lift::MECHANISM_HOMING_ANGLE));
    }

    double height_to_angle(double height_inches) {
        const double sine = (height_inches - constants::lift::MECHANISM_HOMING_POSITION) / EFFECTIVE_LENGTH_INCHES + std::sin(constants::lift::MECHANISM_HOMING_ANGLE);
        const double clamped_sine = std::clamp(sine, std::sin(constants::lift::MECHANISM_HOMING_ANGLE), std::sin(constants::lift::MAXIMUM_ANGLE_RADIANS));

        return std::asin(clamped_sine);
    }

    double angular_to_linear_velocity(double angle_radians, double angular_velocity) {
        // d(height)/dt = effective_length * cos(angle) * d(angle)/dt
        return constants::lift::STAGE_COUNT * constants::lift::STAGE_LENGTH_INCHES * std::cos(angle_radians) * angular_velocity;
    }

    double clamp_angle(double angle_radians) {
        return std::clamp(angle_radians, constants::lift::MECHANISM_HOMING_ANGLE, constants::lift::MAXIMUM_ANGLE_RADIANS);
    }

    double minimum_height() {
        return angle_to_height(constants::lift::MECHANISM_HOMING_ANGLE);
    }

    double maximum_height() {
        return angle_to_height(constants::lift::MAXIMUM_ANGLE_RADIANS);
    }
}