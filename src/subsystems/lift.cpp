#include "subsystems/lift.hpp"
#include "utilities/controls/arm_feedforward.hpp"
#include "utilities/controls/motion_profiler.hpp"
#include "utilities/controls/pid.hpp"
#include "utilities/logger.hpp"
#include "constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

using utilities::Logger;

namespace lift {
    namespace {
        namespace k = constants::lift;

        controls::PIDController controller(k::KP, 0.0, k::KD);
        controls::ArmFeedforward feedforward(k::KS, k::KV, k::KG, k::KA);
        controls::MotionProfiler profiler({k::MAXIMUM_VELOCITY_RADIANS_PER_SECOND, k::MAXIMUM_ACCELERATION_RADIANS_PER_SECOND_SQUARED});

        WantedState wanted_state = WantedState::IDLE;
        WantedState state_after_homing = WantedState::IDLE;
        SystemState system_state = SystemState::IDLING;

        controls::MotionProfiler::State reference{k::MECHANISM_HOMING_ANGLE, 0.0};
        controls::MotionProfiler::State goal{k::MECHANISM_HOMING_ANGLE, 0.0};

        double goal_height_inches = k::MECHANISM_HOMING_POSITION;
        double measured_angle = k::MECHANISM_HOMING_ANGLE;
        double measured_angular_velocity = 0.0;

        bool measurement_valid = false;
        bool needs_resync = false;

        double output_voltage = 0.0;
        double feedforward_voltage = 0.0;
        double feedback_voltage = 0.0;

        bool homed = false;
        uint32_t homing_start_ms = 0;
        std::optional<uint32_t> stall_start_ms;
        uint32_t last_control_ms = 0;

        template <typename T>
        bool average_finite(const std::vector<T>& values, double& result) {
            double sum = 0.0;
            int count = 0;

            for (const T value : values) {
                if (std::isfinite(static_cast<double>(value))) {
                    sum += static_cast<double>(value);
                    ++count;
                }
            }

            if (count == 0) return false;

            result = sum / count;
            return true;
        }

        double radians_to_degrees(double radians) {
            return radians * 180.0 / constants::PI;
        }

        double hold_volts(double angle_radians) {
            return k::KG * std::cos(angle_radians);
        }

        void update_measurements() {
            double motor_rotations = 0.0;
            double motor_rpm = 0.0;

            const bool position_ok = average_finite(globals::lift_motors.get_position_all(), motor_rotations);
            const bool velocity_ok = average_finite(globals::lift_motors.get_actual_velocity_all(), motor_rpm);

            measurement_valid = position_ok && velocity_ok;
            if (!measurement_valid) return;

            measured_angle = kinematics::motor_rotations_to_angle(motor_rotations);
            measured_angular_velocity = kinematics::motor_rpm_to_angular_velocity(motor_rpm);
        }

        void resync_reference() {
            reference = {kinematics::clamp_angle(measured_angle), measured_angular_velocity};
            controller.reset();
        }

        void begin_homing(WantedState after) {
            homed = false;
            stall_start_ms.reset();
            homing_start_ms = pros::millis();
            state_after_homing = (after == WantedState::HOME) ? WantedState::IDLE : after;
            wanted_state = WantedState::HOME;
        }

        SystemState handle_state_transition();

        SystemState update_homing() {
            const uint32_t now = pros::millis();
            const uint32_t elapsed = now - homing_start_ms;

            if (elapsed >= k::HOMING_TIMEOUT_MS) {
                std::printf("Lift: homing timed out\n");
                wanted_state = WantedState::IDLE;
                state_after_homing = WantedState::IDLE;
                return SystemState::IDLING;
            }

            const bool stopped = elapsed >= k::HOMING_SPIN_UP_MS && measurement_valid && std::abs(measured_angular_velocity) <= k::ZERO_VELOCITY_THRESHOLD;

            if (!stopped) {
                stall_start_ms.reset();
                return SystemState::HOMING;
            }

            if (!stall_start_ms.has_value()) {
                stall_start_ms = now;
            }

            if (now - *stall_start_ms < k::ZERO_VELOCITY_TIME_MS) {
                return SystemState::HOMING;
            }

            globals::lift_motors.tare_position_all();
            measured_angle = k::MECHANISM_HOMING_ANGLE;
            measured_angular_velocity = 0.0;
            homed = true;
            stall_start_ms.reset();

            if (state_after_homing == WantedState::IDLE) {
                goal = {constants::lift::MECHANISM_HOMING_ANGLE, 0.0};
                goal_height_inches = constants::lift::MECHANISM_HOMING_POSITION;
                wanted_state = WantedState::MOVE_TO_POSITION;
            } else {
                wanted_state = state_after_homing;
            }

            state_after_homing = WantedState::IDLE;

            return handle_state_transition();
        }

        SystemState handle_state_transition() {
            switch (wanted_state) {
                case WantedState::IDLE:
                    return SystemState::IDLING;

                case WantedState::HOME:
                    return update_homing();

                case WantedState::MOVE_TO_POSITION:
                    if (!homed) {
                        begin_homing(WantedState::MOVE_TO_POSITION);
                        return SystemState::HOMING;
                    }

                    return SystemState::MOVING_TO_POSITION;
            }

            return SystemState::IDLING;
        }

        void on_enter(SystemState state) {
            switch (state) {
                case SystemState::MOVING_TO_POSITION:
                    resync_reference();
                    last_control_ms = pros::millis();
                    break;
                case SystemState::IDLING:
                    globals::lift_motors.move_voltage(0);
                    break;
                case SystemState::HOMING:
                    break;
            }
        }

        void run_voltage(double volts) {
            output_voltage = std::clamp(volts, -12.0, 12.0);
            globals::lift_motors.move_voltage(static_cast<int32_t>(std::lround(output_voltage * 1000.0)));
        }

        void run_position() {
            const uint32_t now = pros::millis();
            double dt = (now - last_control_ms) / 1000.0;

            if (dt <= 0.0) return;
            last_control_ms = now;

            if (!measurement_valid) {
                feedforward_voltage = 0.0;
                feedback_voltage = 0.0;
                needs_resync = true;
                run_voltage(0.0);
                return;
            }

            if (dt > constants::DT_SECONDS || needs_resync) {
                resync_reference();
                needs_resync = false;
                dt = constants::DT_SECONDS;
            }

            const controls::MotionProfiler::State previous = reference;
            reference = profiler.calculate(dt, previous, goal);
            const double reference_acceleration = (reference.velocity - previous.velocity) / dt;

            feedforward_voltage = feedforward.calculate(reference.position, reference.velocity, reference_acceleration);
            feedback_voltage = controller.calculate(measured_angle, reference.position, dt);

            const double voltage = feedforward_voltage + feedback_voltage;

            if (!std::isfinite(voltage)) {
                needs_resync = true;
                run_voltage(0.0);
                return;
            }

            run_voltage(voltage);
        }

        void apply_states() {
            switch (system_state) {
                case SystemState::IDLING:
                    output_voltage = 0.0;
                    feedforward_voltage = 0.0;
                    feedback_voltage = 0.0;
                    globals::lift_motors.move_voltage(0);
                    break;

                case SystemState::HOMING:
                    feedforward_voltage = 0.0;
                    feedback_voltage = 0.0;
                    run_voltage(k::HOMING_VOLTAGE);
                    break;

                case SystemState::MOVING_TO_POSITION:
                    run_position();
                    break;
            }
        }

        const char* wanted_state_name(WantedState state) {
            switch (state) {
                case WantedState::IDLE: return "IDLE";
                case WantedState::HOME: return "HOME";
                case WantedState::MOVE_TO_POSITION: return "MOVE";
            }

            return "?";
        }

        const char* system_state_name(SystemState state) {
            switch (state) {
                case SystemState::IDLING: return "IDLING";
                case SystemState::HOMING: return "HOMING";
                case SystemState::MOVING_TO_POSITION: return "MOVING";
            }

            return "?";
        }

        void log_outputs() {
            double max_temperature = 0.0;
            double total_current = 0.0;

            for (const double t : globals::lift_motors.get_temperature_all()) {
                if (std::isfinite(t)) max_temperature = std::max(max_temperature, t);
            }

            for (const int32_t ma : globals::lift_motors.get_current_draw_all()) {
                if (ma != PROS_ERR) total_current += ma;
            }

            Logger::recordOutput("Lift/CurrentHeightInches", get_current_height());
            Logger::recordOutput("Lift/TargetHeightInches", goal_height_inches);
            Logger::recordOutput("Lift/ReferenceHeightInches", kinematics::angle_to_height(reference.position));

            Logger::recordOutput("Lift/AngleDegrees", radians_to_degrees(measured_angle));
            Logger::recordOutput("Lift/ReferenceAngleDegrees", radians_to_degrees(reference.position));
            Logger::recordOutput("Lift/GoalAngleDegrees", radians_to_degrees(goal.position));
            Logger::recordOutput("Lift/CosAngle", std::cos(measured_angle));

            Logger::recordOutput("Lift/AngularVelocityRadPerSecond", measured_angular_velocity);
            Logger::recordOutput("Lift/ReferenceAngularVelocityRadPerSecond", reference.velocity);
            Logger::recordOutput("Lift/LinearVelocityInchesPerSecond", get_linear_velocity());

            Logger::recordOutput("Lift/OutputVoltage", output_voltage);
            Logger::recordOutput("Lift/FeedforwardVoltage", feedforward_voltage);
            Logger::recordOutput("Lift/FeedbackVoltage", feedback_voltage);

            Logger::recordOutput("Lift/MaxTemperatureCelsius", max_temperature);
            Logger::recordOutput("Lift/TotalCurrentMilliamps", total_current);

            Logger::recordOutput("Lift/WantedState", static_cast<int>(wanted_state));
            Logger::recordOutput("Lift/SystemState", static_cast<int>(system_state));
            Logger::recordOutput("Lift/Homed", homed);
        }
    }

    void initialize() {
        globals::lift_motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
        globals::lift_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

        const char* outputs[] = {
            "Lift/CurrentHeightInches", 
            "Lift/TargetHeightInches", 
            "Lift/ReferenceHeightInches",
            "Lift/AngleDegrees", 
            "Lift/ReferenceAngleDegrees", 
            "Lift/GoalAngleDegrees", 
            "Lift/CosAngle",
            "Lift/AngularVelocityRadPerSecond", 
            "Lift/ReferenceAngularVelocityRadPerSecond", 
            "Lift/LinearVelocityInchesPerSecond",
            "Lift/OutputVoltage", 
            "Lift/FeedforwardVoltage", 
            "Lift/FeedbackVoltage", 
            "Lift/ProfilingVoltage",
            "Lift/MaxTemperatureCelsius", 
            "Lift/TotalCurrentMilliamps",
            "Lift/WantedState", 
            "Lift/SystemState", 
            "Lift/Homed",
        };

        for (const char* name : outputs) {
            Logger::registerOutput(name);
        }
    }

    void periodic() {
        update_measurements();

        const SystemState next_state = handle_state_transition();

        if (next_state != system_state) {
            on_enter(next_state);
        }

        system_state = next_state;
        apply_states();
        log_outputs();
    }

    void set_wanted_state(WantedState state) {
        if (state == wanted_state) return;

        if (state == WantedState::HOME) {
            begin_homing(WantedState::IDLE);
            return;
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

        goal = {kinematics::height_to_angle(height_inches), 0.0};
        goal_height_inches = kinematics::angle_to_height(goal.position);

        if (wanted_state == WantedState::HOME) {
            state_after_homing = WantedState::MOVE_TO_POSITION;
        } else {
            set_wanted_state(WantedState::MOVE_TO_POSITION);
        }
    }

    double get_target_height() {
        return goal_height_inches;
    }

    double get_current_height() {
        return kinematics::angle_to_height(measured_angle);
    }

    double get_angle() {
        return measured_angle;
    }

    double get_angular_velocity() {
        return measured_angular_velocity;
    }

    double get_linear_velocity() {
        return kinematics::angular_to_linear_velocity(measured_angle, measured_angular_velocity);
    }

    bool is_homed() {
        return homed;
    }

    bool at_setpoint() {
        if (!homed || !measurement_valid || system_state != SystemState::MOVING_TO_POSITION) {
            return false;
        }

        const bool profile_finished = reference.position == goal.position && reference.velocity == goal.velocity;

        return profile_finished && std::abs(goal_height_inches - get_current_height()) <= k::POSITION_TOLERENCE_INCHES && std::abs(get_linear_velocity()) <= k::VELOCITY_TOLERENCE_INCHES_PER_SECOND;
    }

    void screen_subtask() {
        ez::screen_print(std::string("Lift: ") + wanted_state_name(wanted_state) + " -> " + system_state_name(system_state), 1);
        ez::screen_print("Height: " + util::to_string_with_precision(get_current_height()) + " / " + util::to_string_with_precision(goal_height_inches) + " in", 2);
        ez::screen_print("Angle: " + util::to_string_with_precision(radians_to_degrees(measured_angle), 1) + " deg  cos: " + util::to_string_with_precision(std::cos(measured_angle), 3), 3);
        ez::screen_print("Velocity: " + util::to_string_with_precision(measured_angular_velocity) + " rad/s  ref " + util::to_string_with_precision(reference.velocity), 4);
        ez::screen_print("Output: " + util::to_string_with_precision(output_voltage) + util::to_string_with_precision(feedforward_voltage) + util::to_string_with_precision(feedback_voltage), 5);
        // ez::screen_print("Profiling Voltage: " + util::to_string_with_precision(test_voltage) + " / " + util::to_string_with_precision(hold_volts(measured_angle)), 6);
    }
}
