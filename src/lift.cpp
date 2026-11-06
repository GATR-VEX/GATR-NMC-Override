/**
 * @file lift.cpp
 * @brief Lift control logic for DR4B using PID positioning.
 * with a 1:3 external gear ratio
 */

#include "main.h"
#include "subsystems.hpp"
#include <cmath>

// Height presets (in motor encoder degrees - placeholder values!)
// With a 100 RPM red motor and 1:3 gear ratio:
// 1 full arm rotation (360°) = 3 motor revolutions = 1080 motor degrees.
// A typical DR4B arm swings ~60-80 degrees:
const int BASE_HEIGHT = 200;       // Fully lowered / resting height
const int HEIGHT_A    = 380;     // Position 1
const int HEIGHT_B    = 560;     // Position 2

/// PID controller instance for DR4B lift
ez::PID liftPID{0.15, 0.0, 0.0, 0.0, "DR4B Lift"};

/**
 * @brief Helper to get the current DR4B lift position in degrees.
 * Uses the integrated encoder of the dr4b motor group.
 */
static double get_lift_position() {
  return dr4b.get_position();
}

/**
 * @brief DR4B Lift controller task loop.
 */
void LiftController() {
  // Preset base target position
  liftPID.target_set(BASE_HEIGHT);

  // Block this task until signaled to begin by autonomous() or opcontrol()
  while (pros::Task::notify_take(true, TIMEOUT_MAX));

  bool manual_active = false;
  bool display_enabled = true; // Tracks whether controller display is on/off

  while (true) {
    double current_pos = get_lift_position();
    double pid_output = 0;

    // Toggle controller display on/off using E_CONTROLLER_DIGITAL_UP
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      display_enabled = !display_enabled;
      if (!display_enabled) {
        // Clear specific lines so the controller can return to its default system display
        master.clear_line(0);
        master.clear_line(1);
      }
    }

    if (!pros::competition::is_autonomous()) {
      // 1. Manual Controls (L1 = UP, L2 = DOWN)
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        manual_active = true;
        dr4b.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        pid_output = 127;                          // Full speed up
        dr4b.move(pid_output);
        liftPID.target_set(current_pos);          // Follow position so PID holds immediately upon release
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        manual_active = true;
        dr4b.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        pid_output = -127 / 4;                     // Speed down
        dr4b.move(pid_output);
        liftPID.target_set(current_pos);          // Follow position so PID holds immediately upon release
      } else {
        // If manual buttons were just released, lock current position into PID
        if (manual_active) {
          liftPID.target_set(current_pos);
          manual_active = false;
        }

        // 2. Preset Toggle Buttons: Y and B
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
          if (std::abs(liftPID.target_get() - HEIGHT_A) < 20) {
            liftPID.target_set(BASE_HEIGHT);
          } else {
            liftPID.target_set(HEIGHT_A);
          }
        }
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
          if (std::abs(liftPID.target_get() - HEIGHT_B) < 20) {
            liftPID.target_set(BASE_HEIGHT);
          } else {
            liftPID.target_set(HEIGHT_B);
          }
        }

        // Apply PID output to hold or reach target
        pid_output = liftPID.compute(current_pos);

        // Cap downward speed so lift lowers smoothly
        const double MAX_DOWN_SPEED = -30;
        if (pid_output < MAX_DOWN_SPEED) {
          pid_output = MAX_DOWN_SPEED;
        }

        // If PID compensation is less than 10 with no manual actions, use motor hold mode instead of fighting it
        if (std::abs(pid_output) < 10) {
          dr4b.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
          dr4b.brake();
        } else {
          dr4b.move(pid_output);
        }
      }
    } else {
      // In autonomous mode, run PID towards target
      pid_output = liftPID.compute(current_pos);

      const double MAX_DOWN_SPEED = -30;
      if (pid_output < MAX_DOWN_SPEED) {
        pid_output = MAX_DOWN_SPEED;
      }

      // Apply hold if compensation is less than 10
      if (std::abs(pid_output) < 10) {
        dr4b.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        dr4b.brake();
      } else {
        dr4b.move(pid_output);
      }
    }

    // Display lift position and PID output to the V5 Controller screen if enabled
    static int print_timer = 0;
    if (display_enabled) {
      if (print_timer % 20 == 0) {
        master.print(0, 0, "Pos:%.0f Tgt:%.0f   ", current_pos, liftPID.target_get());
      } else if (print_timer % 20 == 10) {
        master.print(1, 0, "Out: %.0f          ", pid_output);
      }
    }
    print_timer++;

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task LiftTask(LiftController);

/**
 * @brief Waits until the lift reaches a specified target position.
 */
void wait_lift(int position) {
  liftPID.target_set(position);

  while (liftPID.exit_condition(true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

/**
 * @brief Sends lift to a specified target without blocking.
 */
void async_lift(int position) {
  liftPID.target_set(position);
}
