#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor left_motor(1);
inline pros::Motor right_motor(1);
inline pros::adi::Pneumatics piston({1, 'a'}, false);

// inline pros::adi::DigitalIn limit_switch('A');
