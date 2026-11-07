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

inline ez::PID liftPID{0.8, 0.01, 0.8, 0, "Lift"};
// Add this to define when the lift is "done" moving
inline void lift_exit_condition_set() {
    liftPID.exit_condition_set(80, 300, 300, 700, 2000); 
}
// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');


inline void lift_wait() {
  while (liftPID.exit_condition(lift.get_position(), true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}