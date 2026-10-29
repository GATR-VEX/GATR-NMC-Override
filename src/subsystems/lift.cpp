#include "subsystems/lift.hpp"
#include "utilities/logger.hpp"

using utilities::Logger;

namespace lift {
    namespace {
        struct AngularReference {
            double position;
            double velocity;
            double acceleration;
        };

        double radians_to_height(double radians) {
            const double physical_angle = radians + constants::lift::MECHANISM_HOMING_ANGLE;
            return constants::lift::ARM_LENGTH_DR4B * (std::sin(physical_angle) - std::sin(constants::lift::MECHANISM_HOMING_ANGLE));
        }

        double height_to_radians(double height_inches) {
            return std::asin(height_inches / constants::lift::ARM_LENGTH_DR4B + std::sin(constants::lift::MECHANISM_HOMING_ANGLE)) - constants::lift::MECHANISM_HOMING_ANGLE;
        }

        AngularReference linear_to_angular_reference(double height_inches, double linear_velocity, double linear_acceleration) {
            const double length = constants::lift::ARM_LENGTH_DR4B;
            const double position = height_to_radians(height_inches);
            const double physical_angle = position + constants::lift::MECHANISM_HOMING_ANGLE;
            const double cosine = std::cos(physical_angle);
            const double angular_velocity = linear_velocity / (length * cosine);
            const double angular_acceleration = (linear_acceleration + length * std::sin(physical_angle) * angular_velocity * angular_velocity) / (length * cosine);

            return {
                position,
                angular_velocity,
                angular_acceleration
            };
        }

        WantedState wanted_state = WantedState::IDLE;
        SystemState system_state = SystemState::IDLING;
        SystemState next_system_state = SystemState::IDLING;

        controls::PIDController controller = controls::PIDController(constants::lift::KP, 0.0, constants::lift::KD);
        controls::ArmFeedforward feedforward = controls::ArmFeedforward(constants::lift::KS, constants::lift::KV, constants::lift::KG, constants::lift::KA);
        controls::MotionProfiler motion_profiler = controls::MotionProfiler({
            constants::lift::MAXIMUM_VELOCITY_INCHES_PER_SECOND,
            constants::lift::MAXIMUM_ACCELERATION_INCHES_PER_SECOND_SQUARED
        });

        controls::MotionProfiler::State profiled_reference{};
        controls::MotionProfiler::State goal_reference{};

        bool homed = false;

        std::optional<uint32_t> home_timestamp;
        uint32_t homing_start_timestamp = 0;
        uint32_t last_motion_profiler_timestamp = 0;
        
        SystemState handle_state_transitsion() {
            switch (wanted_state) {
                case WantedState::HOME: {
                    const uint32_t now = pros::millis();

                    if (now - homing_start_timestamp >= constants::lift::HOMING_TIMEOUT_MS) {
                        set_wanted_state(WantedState::IDLE);
                        return SystemState::IDLING;
                    }

                    if (std::abs(get_velocity()) <= constants::lift::ZERO_VELOCITY_THRESHOLD) {
                        if (!home_timestamp.has_value()) {
                            home_timestamp = now;
                            return SystemState::HOMING;
                        }

                        if (now - *home_timestamp >= constants::lift::ZERO_VELOCITY_TIME_MS) {
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
                    if (!homed) {
                        set_wanted_state(WantedState::HOME);
                        return SystemState::HOMING;
                    }

                    return SystemState::MOVING_TO_POSITION;
            }

            return SystemState::IDLING;
        }

        void apply_states() {
            switch (system_state) {
                case SystemState::IDLING:
                    globals::lift_motors.move(0.0);
                    break;
                case SystemState::HOMING:
                    globals::lift_motors.move_voltage(constants::lift::HOMING_VOLTAGE);
                    break;
                case SystemState::MOVING_TO_POSITION:
                    const uint32_t now = pros::millis();
                    const double dt = (now - last_motion_profiler_timestamp) / 1000.0;

                    if (dt <= 0.0) return;
                    last_motion_profiler_timestamp = now;

                    if (dt > 0.1) {
                        const double angle = get_position();
                        const double physical_angle = angle + constants::lift::MECHANISM_HOMING_ANGLE;
                        profiled_reference = {radians_to_height(angle), constants::lift::ARM_LENGTH_DR4B * std::cos(physical_angle) * get_velocity()};
                        controller.reset();
                        globals::lift_motors.move_voltage(0);
                        return;
                    }

                    const auto previous_reference = profiled_reference;
                    const auto next_reference = motion_profiler.calculate(dt, previous_reference, goal_reference);
                    const double linear_acceleration = (next_reference.velocity - previous_reference.velocity) / dt;
                    const AngularReference reference = linear_to_angular_reference(next_reference.position, next_reference.velocity, linear_acceleration);

                    if (!std::isfinite(reference.position) || !std::isfinite(reference.velocity) || !std::isfinite(reference.acceleration) || std::abs(std::cos(reference.position + constants::lift::MECHANISM_HOMING_ANGLE)) < 0.1) {
                        globals::lift_motors.move_voltage(0);
                        return;
                    }

                    const double output = controller.calculate(get_position(), reference.position, dt) + feedforward.calculate(reference.position + constants::lift::MECHANISM_HOMING_ANGLE, reference.velocity, reference.acceleration);

                    if (!std::isfinite(output)) {
                        globals::lift_motors.move_voltage(0);
                        return;
                    }

                    globals::lift_motors.move_voltage(static_cast<int>(std::clamp(output, -12.0, 12.0) * 1000.0));
                    profiled_reference = next_reference;
                    break;
            }
        }
    }

    void initialize() {
        globals::lift_motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);

        Logger::registerOutput("Lift/CurrentHeightInches");
        Logger::registerOutput("Lift/TargetHeightInches");

        Logger::registerOutput("Lift/CurrentVelocityInchesPerSecond");
        Logger::registerOutput("Lift/TargetVelocityInchesPerSecond");

        Logger::registerOutput("Lift/WantedState");
        Logger::registerOutput("Lift/SystemState");
        Logger::registerOutput("Lift/Homed");
    }

    void periodic() {
        next_system_state = handle_state_transitsion();

        if (next_system_state != system_state && next_system_state == SystemState::MOVING_TO_POSITION) {
            controller.reset();
            const double current_position = get_position();
            const double physical_angle = current_position + constants::lift::MECHANISM_HOMING_ANGLE;
            profiled_reference = {radians_to_height(current_position), constants::lift::ARM_LENGTH_DR4B * std::cos(physical_angle) * get_velocity()};

            last_motion_profiler_timestamp = pros::millis();
        }

        system_state = next_system_state;
        apply_states();

        Logger::recordOutput("Lift/CurrentHeightInches", radians_to_height(get_position()));
        Logger::recordOutput("Lift/TargetHeightInches", goal_reference.position);

        Logger::recordOutput("Lift/CurrentVelocityInchesPerSecond", (constants::lift::ARM_LENGTH_DR4B * std::cos(get_position() + constants::lift::MECHANISM_HOMING_ANGLE) * get_velocity()));
        Logger::recordOutput("Lift/TargetVelocityInchesPerSecond", profiled_reference.velocity);

        Logger::recordOutput("Lift/WantedState", std::to_string(static_cast<int>(wanted_state)));
        Logger::recordOutput("Lift/SystemState", std::to_string(static_cast<int>(system_state)));
        Logger::recordOutput("Lift/Homed", is_homed());
    }

    void set_wanted_state(WantedState state) {
        if (wanted_state == state) return;

        if (state == WantedState::HOME) {
            homed = false;
            home_timestamp.reset();
            homing_start_timestamp = pros::millis();
        }

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
        if (height_inches < constants::lift::MINIMUM_POSITION_INCHES || height_inches > constants::lift::MAXIMUM_POSITION_INCHES) return;

        goal_reference = {height_inches, 0.0};
        set_wanted_state(WantedState::MOVE_TO_POSITION);
    }

    double get_current_height() {
        return radians_to_height(get_position());
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

        return helpers::rotations_to_radians((position_sum / all_positions.size()) / constants::lift::MECHANISM_REDUCTION);
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
        return helpers::rotations_to_radians(averageMotorRPS / constants::lift::MECHANISM_REDUCTION);
    }

    double get_linear_velocity() {
        const double physical_angle = get_position() + constants::lift::MECHANISM_HOMING_ANGLE;
        return constants::lift::ARM_LENGTH_DR4B * std::cos(physical_angle) * get_velocity();
    }

    bool is_homed() {
        return homed;
    }

    
    bool at_setpoint() {
        if (!homed ||
            system_state != SystemState::MOVING_TO_POSITION) {
            return false;
        }

        const double angle = get_position();
        const double physical_angle = angle + constants::lift::MECHANISM_HOMING_ANGLE;
        const double current_height = radians_to_height(angle);
        const double current_linear_velocity = constants::lift::ARM_LENGTH_DR4B * std::cos(physical_angle) * get_velocity();
        const bool profile_finished = std::abs(profiled_reference.position - goal_reference.position) < 1e-4 && std::abs(profiled_reference.velocity) < 1e-4;

        return profile_finished && std::abs(goal_reference.position - current_height) <= constants::lift::POSITION_TOLERENCE_INCHES && std::abs(current_linear_velocity) <= constants::lift::VELOCITY_TOLERENCE_INCHES_PER_SECOND;
    }


    void screen_subtask() {
	    ez::screen_print("Wanted State: " + static_cast<int>(wanted_state), 1);
	    ez::screen_print("System State: " + static_cast<int>(system_state), 2);

        ez::screen_print("Lift Voltage: " + util::to_string_with_precision(globals::lift_motors.get_voltage()), 4);
        ez::screen_print("Target Height: " + util::to_string_with_precision(goal_reference.position), 5);
        ez::screen_print("Current Height: " + util::to_string_with_precision(get_current_height()), 6);
        ez::screen_print("Current Velocity: " + util::to_string_with_precision(get_linear_velocity()), 7);
        ez::screen_print("Target Velocity: " + util::to_string_with_precision(profiled_reference.velocity), 8);
    }
}