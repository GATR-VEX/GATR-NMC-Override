#include "subsystems/drivetrain.hpp"

namespace drivetrain {
    namespace {
        WantedState wantedState = WantedState::IDLE;
        SystemState systemState = SystemState::IDLE;
        Behavior currentBehavior = Behavior::SPLIT_ARCADE;

        double targetPositionX = 0.0;
        double targetPositionY = 0.0;
    
        bool hasTargetPosition = false;

        double getRobotHeading() {
            return globals::chassis.drive_imu_get();
        }

        SystemState handleStateTransition() {
            switch (wantedState) {
                case WantedState::IDLE:
                    globals::chassis.drive_set(0.0, 0.0);
                    return SystemState::IDLE;
                case WantedState::TELEOP_DRIVE:
                    return SystemState::TELEOP_DRIVE;
                case WantedState::DRIVE_TO_POINT:
                    return SystemState::DTP_ALIGNING;
            }

            return SystemState::IDLE;
        }

        void applyStates() {
            switch (systemState) {
                case SystemState::IDLE:
                    break;
                case SystemState::TELEOP_DRIVE:
                    switch (currentBehavior) {
                        case Behavior::TANK:
                            globals::chassis.opcontrol_tank();
                            break;
                        case Behavior::SPLIT_ARCADE:
                            globals::chassis.opcontrol_arcade_standard(ez::SPLIT);
                            break;
                        case Behavior::SINGLE_ARCADE:
                            globals::chassis.opcontrol_arcade_standard(ez::SINGLE);
                            break;
                    }

                    break;
                case SystemState::DTP_ALIGNING:
                    break;
                case SystemState::DTP_ALIGNED:
                    break;
            }
        }

        void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
            std::string tracker_value = "", tracker_width = "";

            if (tracker != nullptr) {
                tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
                tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
            }

            ez::screen_print(tracker_value + tracker_width, line);
        }

        void default_constants() {
            globals::chassis.pid_drive_constants_set(20.0, 0.0, 100.0); // Fwd/rev constants, used for odom and non odom motions
            globals::chassis.pid_heading_constants_set(11.0, 0.0, 20.0); // Holds the robot straight while going forward without odom
            globals::chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0); // Turn in place constants
            globals::chassis.pid_swing_constants_set(6.0, 0.0, 65.0); // Swing constants
            globals::chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5); // Angular control for odom motions
            globals::chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5); // Angular control for boomerang motions

            // Exit conditions
            globals::chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
            globals::chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
            globals::chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
            globals::chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
            globals::chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
            globals::chassis.pid_turn_chain_constant_set(3_deg);
            globals::chassis.pid_swing_chain_constant_set(5_deg);
            globals::chassis.pid_drive_chain_constant_set(3_in);

            // Slew constants
            globals::chassis.slew_turn_constants_set(3_deg, 70);
            globals::chassis.slew_drive_constants_set(3_in, 70);
            globals::chassis.slew_swing_constants_set(3_in, 80);

            // The amount that turns are prioritized over driving in odom motions - if you have tracking wheels, you can run this higher.  1.0 is the max
            globals::chassis.odom_turn_bias_set(0.9);

            globals::chassis.odom_look_ahead_set(7_in); // This is how far ahead in the path the robot looks at
            globals::chassis.odom_boomerang_distance_set(16_in); // This sets the maximum distance away from target that the carrot point can be
            globals::chassis.odom_boomerang_dlead_set(0.625); // This handles how aggressive the end of boomerang motions are

            globals::chassis.pid_angle_behavior_set(ez::shortest); // Changes the default behavior for turning, this defaults it to the shortest path there
        }

    }

    void initialize() {
        // Manual driving behavior
        globals::chassis.opcontrol_drive_activebrake_set(0.0);
        globals::chassis.opcontrol_curve_default_set(0.0, 0.0);
        globals::chassis.opcontrol_drive_reverse_set(true);
        globals::chassis.opcontrol_speed_max_set(127);

        // Joystick curve adjustment buttons
        globals::chassis.opcontrol_curve_buttons_left_set(
            pros::E_CONTROLLER_DIGITAL_LEFT,
            pros::E_CONTROLLER_DIGITAL_RIGHT
        );

        globals::chassis.opcontrol_curve_buttons_right_set(
            pros::E_CONTROLLER_DIGITAL_Y,
            pros::E_CONTROLLER_DIGITAL_A
        );

        // Configure autonomous PID constants.
        default_constants();

        // Initialize EZ-Template chassis and IMU.
        globals::chassis.initialize();

        // Reset subsystem state.
        wantedState = WantedState::IDLE;
        systemState = SystemState::IDLE;
        currentBehavior = Behavior::SPLIT_ARCADE;
    }

    void prepareTeleop() {
        globals::chassis.drive_brake_set(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
        setWantedState(WantedState::TELEOP_DRIVE);
    }

    void prepareAutonomous() {
        globals::chassis.pid_targets_reset();
        globals::chassis.drive_imu_reset();
        globals::chassis.drive_sensor_reset();
        globals::chassis.odom_xyt_set(0_in, 0_in, 0_deg);
        globals::chassis.drive_brake_set(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
    }

    void periodic() {
        systemState = handleStateTransition();
        applyStates();
    }

    void setWantedState(WantedState state) {
        if (wantedState == state) return;
        wantedState = state;
    }

    WantedState getWantedState() {
        return wantedState;
    }

    SystemState getSystemState() {
        return systemState;
    }

    void setBehavior(Behavior behavior) {
        currentBehavior = behavior;
    }

    Behavior getBehavior() {
        return currentBehavior;
    }

    bool isIMUCalibrating() {
        return globals::imu.is_calibrating();
    }

    bool isIMUCalibrated() {
        return globals::chassis.drive_imu_calibrated();
    }

    void setTargetPosition(double x, double y) {
        if(!std::isfinite(x) || !std::isfinite(y)) return;

        targetPositionX = x;
        targetPositionY = y;
        hasTargetPosition = true;

        setWantedState(WantedState::DRIVE_TO_POINT);
    }

    void updatePID() {
        if (pros::competition::is_connected()) {
            if (globals::chassis.pid_tuner_enabled()) {
                globals::chassis.pid_tuner_disable();
            }

            return;
        }

        if (master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X)) {
            globals::chassis.pid_tuner_toggle();
        }

        globals::chassis.pid_tuner_iterate();
    }


    void printDebugScreen() {
        if (!globals::chassis.odom_enabled() || globals::chassis.pid_tuner_enabled()) return;

        ez::screen_print(
            "x: " + util::to_string_with_precision(globals::chassis.odom_x_get()) +
            "\ny: " + util::to_string_with_precision(globals::chassis.odom_y_get()) +
            "\na: " + util::to_string_with_precision(globals::chassis.odom_theta_get()),
            1
        );
    }
}