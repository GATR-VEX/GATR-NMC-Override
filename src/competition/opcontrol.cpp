#include "competition/opcontrol.hpp"
#include "competition/autonomous.hpp"
#include "main.h"
#include "globals.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/superstructure.hpp"
#include "utilities/logger.hpp"

using utilities::Logger;

namespace competition {
    namespace {
        void ez_template_extras() {
            if (!pros::competition::is_connected()) {
                if (master.get_digital_new_press(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
                    autonomous();
                    drivetrain::prepare_teleop();
                }
            }

            drivetrain::update_pid();
        }
    }

    void configure_bindings() {
        using globals::primary;
        using globals::secondary;

        primary.l1().on_true([] {
            //lift::set_target_height(superstructure::get_scoring_height());
            lift::set_target_height(35.0);
        });

        primary.l2().on_true([] {
            lift::set_wanted_state(lift::WantedState::HOME);
        });


        primary.r1().on_true([] {
            intake::set_wanted_state(intake::get_wanted_state() == intake::WantedState::CLOSED ? intake::WantedState::AUTO_INTAKE : intake::WantedState::CLOSED);
        });

        primary.r2().on_true(drivetrain::toggle_slow_mode);

        secondary.right().on_true(superstructure::next_goal);
        secondary.left().on_true(superstructure::previous_goal);
        secondary.up().on_true(superstructure::increase_stack);
        secondary.down().on_true(superstructure::decrease_stack);
        secondary.a().on_true(superstructure::next_game_piece);
        secondary.b().on_true(superstructure::previous_game_piece);
    }

    void opcontrol() {
        Logger::start("teleop");

        drivetrain::prepare_teleop();
        drivetrain::set_behavior(drivetrain::Behavior::SPLIT_ARCADE);

        if (!lift::is_homed()) {
            lift::set_wanted_state(lift::WantedState::HOME);
        }

        globals::secondary.redraw_display();
        //double mv = 0.0;

        while (true) {
            globals::bindings.poll();

            superstructure::write_display(globals::secondary);
            globals::secondary.update_display();

            ez_template_extras();

            // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) mv += 100;
            // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) mv -= 100;
            // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) mv += 20;
            // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) mv -= 20;

            // globals::lift_motors.move_voltage(mv);

            // printf("Motor voltage: %.2f \n", mv);

            pros::delay(ez::util::DELAY_TIME);
        }
    }
}
