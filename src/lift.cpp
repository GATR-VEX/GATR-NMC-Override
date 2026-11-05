/**
 * @file lift.cpp
 * @brief Lift control logic for DR4B using PID positioning.

 * with a 1:3 external gear ratio

 * - Button A: Moves to HEIGHT_A regardless of current height (press again to return to BASE_HEIGHT)
 * - Button B: Moves to HEIGHT_B regardless of current height (press again to return to BASE_HEIGHT)
 */

#include "main.h"
#include "subsystems.hpp"
#include <cmath>

// Height presets (in motor encoder degrees - placeholder values!)
// With a 100 RPM red motor and 1:3 gear ratio:
// 1 full arm rotation (360°) = 3 motor revolutions = 1080 motor degrees.
// A typical DR4B arm swings ~60-80 degrees:
const int BASE_HEIGHT = 150;       // Fully lowered / resting height
const int HEIGHT_A    = 400;     // Position 1
const int HEIGHT_B    = 800;     // Position 2

/// PID controller instance for DR4B lift
/// @param kP Proportional gain 
/// @param kI Integral gain
/// @param kD Derivative gain
/// @param start_i Start integral threshold
/// @param name PID name for debugging
ez::PID liftPID{0.15, 0.0, 0.3, 0.0, "DR4B Lift"};

/**
 * @brief Helper to get the current DR4B lift position in degrees.
 * Uses the integrated encoder of the dr4b motor group.
 */
static double get_lift_position() {
  return dr4b.get_position();
}

/**
 * @brief DR4B Lift controller task loop.
 *
 * Handles teleop inputs (L1/L2 manual jog and A/B preset toggles)
 * and continuously applies PID motor power to dr4b.
 */
void LiftController() {
  // Preset base target position
  liftPID.target_set(BASE_HEIGHT);

  // Block this task until signaled to begin by autonomous() or opcontrol()
  while (pros::Task::notify_take(true, TIMEOUT_MAX));

  bool manual_active = false;

  while (true) {
    if (!pros::competition::is_autonomous()) {
      // 1. Manual Controls (L1 = UP, L2 = DOWN)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        manual_active = true;
        dr4b.move(127);                          // Full speed up
        liftPID.target_set(get_lift_position()); // Follow position so PID holds immediately upon release
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        manual_active = true;
        dr4b.move(-127/4);                         // Speed down
        liftPID.target_set(get_lift_position()); // Follow position so PID holds immediately upon release
      } else {
        // If manual buttons were just released, lock current position into PID
        if (manual_active) {
          liftPID.target_set(get_lift_position());
          manual_active = false;
        }

        // 2. Preset Toggle Buttons: A and B
        // When A is toggled: moves to HEIGHT_A regardless of current height.
        // If already at/targeting HEIGHT_A, toggles back to BASE_HEIGHT.
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
          if (std::abs(liftPID.target_get() - HEIGHT_A) < 20) {
            liftPID.target_set(BASE_HEIGHT);
            master.rumble("..");  // Double rumble: returning to base
          } else {
            liftPID.target_set(HEIGHT_A);
            master.rumble(".");   // Single rumble: preset A
          }
        }
        // When B is toggled: moves to HEIGHT_B regardless of current height.
        // If already at/targeting HEIGHT_B, toggles back to BASE_HEIGHT.
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
          if (std::abs(liftPID.target_get() - HEIGHT_B) < 20) {
            liftPID.target_set(BASE_HEIGHT);
            master.rumble("..");  // Double rumble: returning to base
          } else {
            liftPID.target_set(HEIGHT_B);
            master.rumble(".");   // Single rumble: preset B
          }
        }

        // Apply PID output to hold or reach target
        dr4b.move(liftPID.compute(get_lift_position()));
      }
    } else {
      // In autonomous mode, run PID towards target
      dr4b.move(liftPID.compute(get_lift_position()));
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task LiftTask(LiftController);

/**
 * @brief Waits until the lift reaches a specified target position.
 *
 * Blocking function for autonomous routines that halts execution
 * until the PID controller exits according to its configured exit conditions.
 *
 * @param position Target lift position in encoder degrees
 */
void wait_lift(int position) {
  liftPID.target_set(position);

  // Wait on the lift PID exit condition
  while (liftPID.exit_condition(true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

/**
 * @brief Sends lift to a specified target without blocking.
 *
 * @param position Target lift position in encoder degrees
 */
void async_lift(int position) {
  liftPID.target_set(position);
}
