#include "competition/opcontrol.hpp"
#include "competition/autonomous.hpp"
#include "main.h"
#include "globals.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/superstructure.hpp"
#include "utilities/logger.hpp"
#include "constants.hpp"

#include <algorithm>

using utilities::Logger;

namespace competition {
    namespace {
		std::uint32_t opcontrol_start_ms = 0;

		int seconds_left() {
			const int elasped_time = static_cast<int>(pros::millis() - opcontrol_start_ms);
			const int time_left_ms = constants::OPCONTROL_LENGTH_SECONDS * 1000 - elasped_time;

			return std::max(0, (time_left_ms + 999) / 1000);
		}

        void ez_template_extras() {
            if (!pros::competition::is_connected()) {
                if (master.get_digital_new_press(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
                    autonomous();
                    drivetrain::prepare_teleop();
                }
            }

            drivetrain::update_pid();
        }

        bool lift_is_down() {
            return lift::is_homed() && lift::get_current_height() <= lift::kinematics::minimum_height() + constants::lift::LIFT_DOWN_MARGIN_IN;
        }
    }

    void configure_bindings() {
        using globals::primary;
        using globals::secondary;

        primary.l1().on_true([] {
            if (superstructure::has_scoring_height()) {
				lift::set_target_height(superstructure::get_scoring_height());
			}
        });

        primary.l2().on_true([] {
            lift::set_wanted_state(lift::WantedState::HOME);
        });


        primary.r1().on_true([] {
            intake::set_wanted_state(intake::WantedState::CLOSED);
        });

        primary.r2().on_true([] {
            intake::set_wanted_state(lift_is_down() ? intake::WantedState::AUTO_INTAKE : intake::WantedState::OPEN);
        });

        utilities::Trigger lift_down(globals::bindings, [] { 
            return lift_is_down(); 
        });

        lift_down.on_true([] {
            if (intake::get_wanted_state() == intake::WantedState::OPEN) {
                intake::set_wanted_state(intake::WantedState::AUTO_INTAKE);
            }
        });

        primary.a().on_true(drivetrain::toggle_slow_mode);

		secondary.up().on_true(superstructure::next_node);
		secondary.down().on_true(superstructure::previous_node);
		secondary.r1().on_true(superstructure::increase_stack);
		secondary.r1().on_true(superstructure::decrease_stack);

        utilities::Trigger tipping(globals::bindings, [tipped_since = std::uint32_t{0}, was_tipped = false]() mutable {
            const double pitch = globals::imu.get_roll();
            const bool tipped = std::isfinite(pitch) && std::abs(pitch) > constants::drivetrain::TIP_PITCH_DEGREES;

            if (!tipped) {
                was_tipped = false;
                return false;
            }

            if (!was_tipped) {
                was_tipped = true;
                tipped_since = pros::millis();
            }

            return pros::millis() - tipped_since >= constants::drivetrain::TIP_CONFIRM_MS;
        });

        tipping.on_true([] {
            lift::set_wanted_state(lift::WantedState::HOME);
            globals::primary.rumble("- .");
        });
    }

    void opcontrol() {
        Logger::start(constants::TUNING_MODE ? "practice" : "teleop");
		opcontrol_start_ms = pros::millis();

        drivetrain::prepare_teleop();
        drivetrain::set_behavior(drivetrain::Behavior::SPLIT_ARCADE);

        if (!lift::is_homed()) {
            lift::set_wanted_state(lift::WantedState::HOME);
        }

        globals::secondary.redraw_display();

        while (true) {
            globals::bindings.poll();

            superstructure::write_display(globals::secondary, seconds_left());
            globals::secondary.update_display();

            pros::delay(ez::util::DELAY_TIME);
        }
    }
}
