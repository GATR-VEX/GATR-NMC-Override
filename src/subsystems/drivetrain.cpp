#include "subsystems/drivetrain.hpp"
//#include "globals.hpp"

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

    void setTargetPosition(double x, double y) {
        if(!std::isfinite(x) || !std::isfinite(y)) return;

        targetPositionX = x;
        targetPositionY = y;
        hasTargetPosition = true;

        setWantedState(WantedState::DRIVE_TO_POINT);
    }
}