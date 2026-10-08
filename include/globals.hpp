#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

#include "EZ-Template/api.hpp"
#include "api.h"

namespace globals {
    // Drivetrain
    extern ez::Drive chassis;
    extern pros::Imu& imu;

    // Intake
    extern pros::adi::Pneumatics gripper;
}