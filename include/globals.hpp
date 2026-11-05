#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

#include "EZ-Template/api.hpp"
#include "api.h"

#include "utilities/bindings.hpp"

namespace globals {
    // Drivetrain
    extern ez::Drive chassis;
    extern pros::Imu& imu;

    // Lift
    extern pros::MotorGroup lift_motors;

    // Intake
    extern pros::adi::Pneumatics piston;
    extern pros::Distance distance;

    // Controllers
    extern utilities::EventLoop bindings;
    extern utilities::Controller primary;
    extern utilities::Controller secondary;
}