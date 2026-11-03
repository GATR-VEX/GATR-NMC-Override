#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
extern pros::MotorGroup intake;
extern pros::MotorGroup lift;
extern pros::Motor tilt; //operates wrist

extern ez::Piston piston; //claw piston (only piston on bot)


