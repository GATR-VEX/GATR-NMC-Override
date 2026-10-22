#include "subsystems/lift.hpp"

namespace lift {
    namespace {
        WantedState previous_wanted_state = WantedState::IDLE;
        WantedState wanted_state = WantedState::IDLE;
        SystemState system_state = SystemState::IDLING;

        utilities::PIDController controller = utilities::PIDController(constants::lift::kP, 0.0, constants::lift::kD);
        utilities::ArmFeedforward feedforward = utilities::ArmFeedforward(constants::lift::kS, constants::lift::kV, constants::lift::kG, constants::lift::kA);
        double target_position = 0.0;

        bool homed = false;
        bool setpoint = false;

        std::optional<uint32_t> home_timestamp;
        uint32_t homing_start_timestamp = 0;
        uint32_t last_controller_timestamp = 0;

        double radians_to_height(double radians) {
            return constants::lift::kArmLengthInches * std::sin(radians);
        }

        double height_to_radians(double height_inches) {
            return std::asin(height_inches / constants::lift::kArmLengthInches);
        }

        SystemState handle_state_transitsion() {
            switch (wanted_state) {
                case WantedState::HOME: {
                    const uint32_t now = pros::millis();

                    if (system_state != SystemState::HOMING) {
                        homed = false;
                        home_timestamp.reset();
                        homing_start_timestamp = pros::millis();
                    }

                    if (now - homing_start_timestamp >= constants::lift::kHomingTimeoutMs) {
                        set_wanted_state(WantedState::IDLE);
                        return SystemState::IDLING;
                    }

                    if (std::abs(get_velocity()) <= constants::lift::kZeroVelocityThreshold) {
                        if (!home_timestamp.has_value()) {
                            home_timestamp = now;
                            return SystemState::HOMING;
                        }

                        if (now - *home_timestamp >= constants::lift::kZeroVelocityTimeMs) {
                            globals::lift_motors.move(0);
                            globals::lift_motors.tare_position_all();

                            homed = true;
                            home_timestamp.reset();

                            set_wanted_state(WantedState::IDLE);
                            return SystemState::IDLING;
                        }
                    } else {
                        home_timestamp.reset();
                    }

                    return SystemState::HOMING;
                }
                case WantedState::IDLE:
                    return SystemState::IDLING;
                case WantedState::MOVE_TO_POSITION:
                    return homed ? SystemState::MOVING_TO_POSITION : SystemState::IDLING;
            }

            return SystemState::IDLING;
        }

        void apply_states() {
            switch (system_state) {
                case SystemState::IDLING:
                    globals::lift_motors.move(0.0);
                    break;
                case SystemState::HOMING:
                    globals::lift_motors.move_voltage(constants::lift::kHomingPower);
                    break;
                case SystemState::MOVING_TO_POSITION:
                    const uint32_t now = pros::millis();
                    const double dt = (now - last_controller_timestamp) / 1000.0;

                    last_controller_timestamp = now;
                    const double currentPosition = get_position();

                    const double output = controller.calculate(currentPosition, target_position, dt) + feedforward.calculate(0.0, get_velocity());
                    globals::lift_motors.move_voltage(static_cast<int>(std::clamp(output, -12000.0, 12000.0)));

                    break;
            }
        }

		std::string wanted_state_to_string(WantedState state) {
			switch (state) {
				case WantedState::IDLE:
					return "IDLE";
				case WantedState::HOME:
					return "HOME";
				case WantedState::MOVE_TO_POSITION:
					return "MOVE_TO_POSITION";
			}

			return "INDETERMINITE";
		}

		std::string system_state_to_string(SystemState state) {
			switch (state) {
				case SystemState::IDLING:
					return "IDLING";
				case SystemState::HOMING:
					return "HOMING";
				case SystemState::MOVING_TO_POSITION:
					return "MOVING_TO_POSITION";
			}

			return "INDETERMINITE";
		}
    }

    void initialize() {
        globals::lift_motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    }

    void periodic() {
        system_state = handle_state_transitsion();
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

    void set_target_height(double height_inches) {
        if (!std::isfinite(height_inches)) return;
        if (height_inches < 0.0) return;

        target_position = height_to_radians(height_inches);
        set_wanted_state(WantedState::MOVE_TO_POSITION);
    }

    double get_position() {
        vector<double> all_positions = globals::lift_motors.get_position_all();

        if (all_positions.empty()) {
            return 0.0;
        }

        double position_sum = 0.0;

        for (double position: all_positions) {
            position_sum += position;
        }

        return helpers::rotations_to_radians((position_sum / all_positions.size()) / constants::lift::kMotorReduction);
    }

    double get_velocity() {
        vector<double> all_velocities = globals::lift_motors.get_actual_velocity_all();

        if (all_velocities.empty()) {
            return 0.0;
        }

        double velocity_sum = 0.0;

        for (double velocity: all_velocities) {
            velocity_sum += velocity;
        }

        double averageMotorRPS = (velocity_sum / all_velocities.size()) / 60.0;
        return helpers::rotations_to_radians(averageMotorRPS / constants::lift::kMotorReduction);
    }

    bool is_homed() {
        return homed;
    }

    bool at_setpoint() {
        return setpoint;
    }

    void screen_subtask() {
	    ez::screen_print("Wanted State: " + wanted_state_to_string(wanted_state), 1);
	    ez::screen_print("System State: " + system_state_to_string(system_state), 2);
    }
}