#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
extern pros::MotorGroup intake;
extern pros::MotorGroup lift;
extern pros::Motor tilt;
extern ez::Piston piston;
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');