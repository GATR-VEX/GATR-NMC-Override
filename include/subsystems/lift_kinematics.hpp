#pragma once

namespace lift::kinematics {
    double motor_rotations_to_angle(double motor_rotations);
    double motor_rpm_to_angular_velocity(double motor_rpm);
    double angle_to_height(double angle_radians);
    double height_to_angle(double height_inches);
    double angular_to_linear_velocity(double angle_radians, double angular_velocity);
    double clamp_angle(double angle_radians);
    double minimum_height();
    double maximum_height();
}