#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Subsystem Hardware Definitions

// 11W 100 RPM (red gearset) DR4B lift motors with 1:3 gear ratio
inline pros::MotorGroup dr4b({11, -12}, pros::MotorGearset::red);  
inline pros::Motor claw(13);

// inline pros::adi::DigitalIn limit_switch('A');

// DR4B Lift PID & Control Declarations

// Height presets (placeholder values in motor degrees - tune for your robot!)
// DR4B with 100 RPM red motors & 1:3 gear ratio
extern const int BASE_HEIGHT;
extern const int HEIGHT_A;
extern const int HEIGHT_B;

// PID controller, state, and background task
extern bool pid_enabled;
extern ez::PID liftPID;
extern pros::Task LiftTask;

/// Starts the lift control task loop
void LiftController();

/// Autonomous helper: sets lift target and blocks until reached
void wait_lift(int position);

/// Autonomous helper: sets lift target asynchronously
void async_lift(int position);
