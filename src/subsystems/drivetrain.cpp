#include "subsystems/drivetrain.hpp"
//#include "globals.hpp"

namespace drivetrain {
    namespace {
        constexpr double HEADING_KP = 1.5;
        constexpr double MAX_ANGULAR_ROTATIONAL_VELOCITY = 50;

        WantedState wantedState = WantedState::IDLE;
        SystemState systemState = SystemState::IDLE;
        Behavior currentBehavior = Behavior::SPLIT_ARCADE;

        double targetHeading = 0.0;
        double headingError = 0.0;

        double targetPositionX = 0.0;
        double targetPositionY = 0.0;
        
        bool hasTargetHeading = false;
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
                case WantedState::HEADING_LOCK:
                    return SystemState::HOLDING_HEADING;
                case WantedState::DRIVE_TO_POINT:
                    if (std::abs(headingError) <= 1.0) {
                        return SystemState::DTP_ALIGNED;
                    }

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
                case SystemState::HOLDING_HEADING:
                    int rotationOutput;
                    headingError = wrapHeading(targetHeading - getRobotHeading());

                    if (std::abs(headingError) < 1.0) {
                        rotationOutput = 0;
                    }

                    rotationOutput = std::clamp(
                        static_cast<int>(std::round(std::clamp(
                            headingError * HEADING_KP,
                            -static_cast<double>(MAX_ANGULAR_ROTATIONAL_VELOCITY),
                            static_cast<double>(MAX_ANGULAR_ROTATIONAL_VELOCITY)
                        ))),
                        -127,
                        127
                    );

                    globals::chassis.drive_set(
                        std::clamp<int>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) + rotationOutput, -127, 127),
                        std::clamp<int>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) - rotationOutput, -127, 127)
                    );

                    break;
                case SystemState::DTP_ALIGNING:
                    break;
                case SystemState::DTP_ALIGNED:
                    break;
            }
        }
    }

    void initialize() {
        globals::chassis.opcontrol_drive_activebrake_set(0.0);
        globals::chassis.opcontrol_curve_default_set(0.0, 0.0);
        globals::chassis.opcontrol_drive_reverse_set(true);
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

    void setTargetHeading(double degrees) {
        if(!std::isfinite(degrees)) return;

        targetHeading = std::remainder(degrees, 360.0);
        hasTargetHeading = true;

        setWantedState(WantedState::HEADING_LOCK);
    }

    void setTargetPosition(double x, double y) {
        if(!std::isfinite(x) || !std::isfinite(y)) return;

        targetPositionX = x;
        targetPositionY = y;
        hasTargetPosition = true;

        setWantedState(WantedState::DRIVE_TO_POINT);
    }
}