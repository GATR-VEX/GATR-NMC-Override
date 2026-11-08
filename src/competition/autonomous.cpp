#include "competition/autonomous.hpp"
#include "main.h"
#include "globals.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/lift.hpp"
#include "utilities/logger.hpp"

using utilities::Logger;

namespace competition {
    void register_autons() {
        default_constants();

        ez::as::auton_selector.autons_add({
            {"Drive\n\nDrive forward and come back", drive_example},
            {"Turn\n\nTurn 3 times.", turn_example},
            {"Drive and Turn\n\nDrive forward, turn, come back", drive_and_turn},
            {"Drive and Turn\n\nSlow down during drive", wait_until_change_speed},
            {"Swing Turn\n\nSwing in an 'S' curve", swing_example},
            {"Motion Chaining\n\nDrive forward, turn, and come back, but blend everything together :D", motion_chaining},
            {"Combine all 3 movements", combining_movements},
            {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
            {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
            {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
            {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
            {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
            {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
        });
    }

    void autonomous() {
        Logger::start("auton");

        drivetrain::prepare_autonomous();

        if (!lift::is_homed()) {
            lift::set_wanted_state(lift::WantedState::HOME);
        }
        
        ez::as::auton_selector.selected_auton_call();
    }
}
