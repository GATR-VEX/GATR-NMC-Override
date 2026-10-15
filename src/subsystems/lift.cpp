#include "subsystems/lift.hpp"

namespace lift {
    namespace {
        WantedState previousWantedState = WantedState::IDLE;
        WantedState wantedState = WantedState::IDLE;
        SystemState systemState = SystemState::IDLING;

        utilities::PIDController controller = utilities::PIDController(constants::lift::kP, 0.0, constants::lift::kD);
        utilities::ElevatorFeedforward feedforward = utilities::ElevatorFeedforward(constants::lift::kS, constants::lift::kV, constants::lift::kG, constants::lift::kA);
        double targetPosition = 0.0;

        bool homed = false;
        bool setpoint = false;

        std::optional<uint32_t> homeTimestamp;
        uint32_t homingStartTimestamp = 0;
        uint32_t lastControllerTimestamp = 0;

        double radiansToHeight(double radians) {
            return constants::lift::kArmLengthInches * std::sin(radians);
        }

        double heightToRadians(double heightInches) {
            return std::asin(heightInches / constants::lift::kArmLengthInches);
        }

        SystemState handleStateTransitsion() {
            switch (wantedState) {
                case WantedState::HOME: {
                    const uint32_t now = pros::millis();

                    if (previousWantedState != WantedState::HOME) {
                        homed = false;
                        homeTimestamp.reset();
                        homingStartTimestamp = pros::millis();
                    }

                    if (now - homingStartTimestamp >= constants::lift::kHomingTimeoutMs) {
                        setWantedState(WantedState::IDLE);
                        return SystemState::IDLING;
                    }

                    if (std::abs(getVelocity()) <= constants::lift::kZeroVelocityThreshold) {
                        if (!homeTimestamp.has_value()) {
                            homeTimestamp = now;
                            return SystemState::HOMING;
                        }

                        if (now - *homeTimestamp >= constants::lift::kZeroVelocityTimeMs) {
                            globals::liftMotors.move(0);
                            globals::liftMotors.tare_position_all();

                            homed = true;
                            homeTimestamp.reset();

                            setWantedState(WantedState::IDLE);
                            return SystemState::IDLING;
                        }
                    } else {
                        homeTimestamp.reset();
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

        void applyStates() {
            switch (systemState) {
                case SystemState::IDLING:
                    globals::liftMotors.move(0.0);
                    break;
                case SystemState::HOMING:
                    globals::liftMotors.move(constants::lift::kHomingPower);
                    break;
                case SystemState::MOVING_TO_POSITION:
                    const uint32_t now = pros::millis();
                    const double dt = (now - lastControllerTimestamp) / 1000.0;

                    lastControllerTimestamp = now;
                    const double currentPosition = getPosition();

                    const double output = controller.calculate(currentPosition, targetPosition, dt) + feedforward.calculate(0.0, 0.0);
                    globals::liftMotors.move_voltage(static_cast<int>(std::clamp(
                        output, 
                        -12000.0,
                        12000.0
                    )));

                    break;
            }
        }
    }

    void initialize() {
        globals::liftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    }

    void periodic() {
        systemState = handleStateTransitsion();
        applyStates();

        previousWantedState = wantedState;
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

    void setTargetHeight(double heightInches) {
        if (!std::isfinite(heightInches)) return;
        if (heightInches < 0.0) return;

        targetPosition = heightToRadians(heightInches);
        setWantedState(WantedState::MOVE_TO_POSITION);
    }

    double getCurrentHeight() {
        return radiansToHeight(targetPosition);
    }

    double getPosition() {
        vector<double> allPositions = globals::liftMotors.get_position_all();

        if (allPositions.empty()) {
            return 0.0;
        }

        double positionSum = 0.0;

        for (double position: allPositions) {
            positionSum += position;
        }

        return helpers::rotationsToRadians((positionSum / allPositions.size()) / constants::lift::kMotorReduction);
    }

    double getVelocity() {
        vector<double> allVelocities = globals::liftMotors.get_actual_velocity_all();

        if (allVelocities.empty()) {
            return 0.0;
        }

        double velocitySum = 0.0;

        for (double position: allVelocities) {
            velocitySum += position;
        }

        double averageMotorRPS = (velocitySum / allVelocities.size()) / 60.0;
        return helpers::rotationsToRadians(averageMotorRPS / constants::lift::kMotorReduction);
    }

    bool isHomed() {
        return homed;
    }

    bool atSetpoint() {
        return setpoint;
    }
}