#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::MotorGroup lift({-11, 12}); // Negative port reverses that motor
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');