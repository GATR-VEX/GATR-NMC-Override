#include "main.h"
#include "robot.hpp"
#include "competition/autonomous.hpp"
#include "competition/opcontrol.hpp"

void initialize() {
  robot::initialize();
}

void disabled() {
  robot::disabled();
}

void competition_initialize() {
  // Runs while disabled before a match, after initialize(). The EZ auton selector is
  // already on the brain screen from robot::initialize().
}

void autonomous() {
  competition::autonomous();
}

void opcontrol() {
  competition::opcontrol();
}