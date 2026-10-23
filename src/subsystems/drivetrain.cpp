#include "subsystems/drivetrain.hpp"

namespace drivetrain {
    namespace {
        WantedState wanted_state = WantedState::IDLE;
        SystemState system_state = SystemState::IDLING;
        Behavior current_behavior = Behavior::SPLIT_ARCADE;

        double targetPositionX = 0.0;
        double targetPositionY = 0.0;
    
        bool hasTargetPosition = false;

        double get_robot_heading() {
            return globals::chassis.drive_imu_get();
        }

        SystemState handle_state_transition() {
            switch (wanted_state) {
                case WantedState::IDLE:
                    return SystemState::IDLING;
                case WantedState::TELEOP_DRIVE:
                    return SystemState::TELEOP_DRIVE;
                case WantedState::DRIVE_TO_POINT:
                    return SystemState::DTP_ALIGNING;
            }

            return SystemState::IDLING;
        }

        void apply_states() {
            switch (system_state) {
                case SystemState::IDLING:
                    globals::chassis.drive_set(0.0, 0.0);
                    break;
                case SystemState::TELEOP_DRIVE:
                    switch (current_behavior) {
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
	
	std::string wanted_state_to_string(WantedState state) {
	    switch (state) {
		case WantedState::IDLE:
		    return "IDLE";
		case WantedState::TELEOP_DRIVE:
		    return "TELEOP_DRIVE";
		case WantedState::DRIVE_TO_POINT:
		    return "DRIVE_TO_POINT";
	    }

	    return "INDETERMINITE";
	}

	std::string system_state_to_string(SystemState state) {
	    switch (state) {
		case SystemState::IDLING:
		    return "IDLING";
		case SystemState::TELEOP_DRIVE:
		    return "TELEOP_DRIVE";
		case SystemState::DTP_ALIGNING:
		    return "DTP_ALIGNING";
	 	case SystemState::DTP_ALIGNED:
		    return "DTP_ALIGNED";
	    }

	    return "INDETERMINITE";
	} 
    }

    void initialize() {
        // Manual driving behavior
        globals::chassis.opcontrol_drive_activebrake_set(0.0);
        globals::chassis.opcontrol_curve_default_set(0.0, 0.0);
        globals::chassis.opcontrol_drive_reverse_set(true);
        globals::chassis.opcontrol_speed_max_set(127);
        globals::chassis.odom_enable(true);

        // Joystick curve adjustment buttons
        globals::chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);
        globals::chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

        // Configure autonomous PID constants.
        default_constants();

        // Initialize EZ-Template chassis and IMU.
        globals::chassis.initialize();

        // Reset subsystem state.
        wanted_state = WantedState::IDLE;
        system_state = SystemState::IDLING;
        current_behavior = Behavior::SPLIT_ARCADE;
    }

    void prepare_teleop() {
        globals::chassis.drive_brake_set(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
        set_wanted_state(WantedState::TELEOP_DRIVE);
    }

    void prepare_autonomous() {
        globals::chassis.pid_targets_reset();
        globals::chassis.drive_imu_reset();
        globals::chassis.drive_sensor_reset();
        globals::chassis.odom_xyt_set(0_in, 0_in, 0_deg);
        globals::chassis.drive_brake_set(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD);
    }

    void periodic() {
        system_state = handle_state_transition();
        apply_states();
    }

    void set_wanted_state(WantedState state) {
        if (wanted_state == state) return;
        wanted_state = state;
    }

    WantedState get_wanted_state() {
        return wanted_state;
    }

    SystemState get_system_state() {
        return system_state;
    }

    void set_behavior(Behavior behavior) {
        current_behavior = behavior;
    }

    Behavior get_behavior() {
        return current_behavior;
    }

    bool is_imu_calibrating() {
        return globals::imu.is_calibrating();
    }

    bool is_imu_calibrated() {
        return globals::chassis.drive_imu_calibrated();
    }

    void set_target_position(double x, double y) {
        if(!std::isfinite(x) || !std::isfinite(y)) return;

        targetPositionX = x;
        targetPositionY = y;
        hasTargetPosition = true;

        set_wanted_state(WantedState::DRIVE_TO_POINT);
    }

    void update_pid() {
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


    void screen_subtask() {
        if (!globals::chassis.odom_enabled() || globals::chassis.pid_tuner_enabled()) return;

	    ez::screen_print("System State: " + system_state_to_string(system_state), 1);
	    ez::screen_print("Wanted State: " + wanted_state_to_string(wanted_state), 2);
        ez::screen_print(
            "Pose Estimator | X: " + util::to_string_with_precision(globals::chassis.odom_x_get(), 4) +
            " | Y: " + util::to_string_with_precision(globals::chassis.odom_y_get(), 3) +
            " | θ: " + util::to_string_with_precision(globals::chassis.odom_theta_get(), 3),
            3
        );
    }
}