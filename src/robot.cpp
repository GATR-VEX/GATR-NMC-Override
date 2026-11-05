#include "main.h"
#include "globals.hpp"
#include "constants.hpp"
#include "competition/autonomous.hpp"
#include "competition/opcontrol.hpp"
#include "subsystems/drivetrain.hpp"
#include "subsystems/lift.hpp"
#include "subsystems/intake.hpp"
#include "subsystems/superstructure.hpp"
#include "utilities/logger.hpp"
#include "robot.hpp"

#include <cstdint>
#include <cstdio>

using utilities::Logger;

namespace robot {
    namespace {
        constexpr std::uint32_t LOOP_PERIOD_MS = 10;

        void subsystem_loop() {
            bool was_enabled = false;
            std::uint32_t wake_time = pros::millis();

            while (true) {
                const bool enabled = !pros::competition::is_disabled();

                if (enabled && !was_enabled) {
                    lift::on_enable();
                }

                was_enabled = enabled;

                if (enabled) {
                    superstructure::periodic();
                    lift::periodic();
                    intake::periodic();
                    drivetrain::periodic();
                }

                pros::Task::delay_until(&wake_time, LOOP_PERIOD_MS);
            }
        }

        // EZ-Template brain screen pages (only shown when not connected to a field).
        void screen_loop() {
            while (true) {
                if (!pros::competition::is_connected()) {
                    if (ez::as::page_blank_is_on(0)) {
                        drivetrain::screen_subtask();
                    } else if (ez::as::page_blank_is_on(1)) {
                        lift::screen_subtask();
                    } else if (ez::as::page_blank_is_on(2)) {
                        intake::screen_subtask();
                    }
                } else if (ez::as::page_blank_amount() > 0) {
                    ez::as::page_blank_remove_all();
                }

                pros::delay(ez::util::DELAY_TIME);
            }
        }
    }

    void initialize() {
        ez::ez_template_print();
        pros::delay(500); 

        Logger::initialize();
        pros::lcd::initialize();

        drivetrain::initialize();
        lift::initialize();
        intake::initialize();
        superstructure::initialize();

        competition::register_autons();
        competition::configure_bindings();

        ez::as::initialize();
        master.rumble(drivetrain::is_imu_calibrated() ? "." : "---");

        // Higher priority than the autonomous / opcontrol tasks so control timing stays steady. Might remove if unstable
        static pros::Task subsystems(subsystem_loop, TASK_PRIORITY_DEFAULT + 1, TASK_STACK_DEPTH_DEFAULT, "Subsystems");
        static pros::Task screen(screen_loop, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Screen");
    }

    void disabled() {
        if (!Logger::stop()) {
            std::printf("Logger: Failed to close recording\n");
        }
    }
}