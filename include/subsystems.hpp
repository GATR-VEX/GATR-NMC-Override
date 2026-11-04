#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

extern pros::Controller master;
extern pros::MotorGroup lift;
extern pros::Motor claw;
extern int liftp;
extern int liftn;

inline void set_lift(int input) {
  lift.move(input);
}

inline ez::PID liftPID{0.55, 0, 0.8, 0, "Lift"};

// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');


inline void lift_wait() {
  while (liftPID.exit_condition(lift.get_position(), true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}