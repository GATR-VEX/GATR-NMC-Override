#include "subsystems/intake.hpp"

namespace intake {
    namespace {
        WantedState wantedState = WantedState::CLOSED;
        SystemState systemState = SystemState::CLOSED_UNLOADED;

        SystemState handleStateTransition() {
            switch (wantedState) {
                case WantedState::CLOSED:
                    return SystemState::CLOSED_LOADED;
                case WantedState::OPEN:
                    return SystemState::OPEN;
            }

            return SystemState::CLOSED_UNLOADED;
        }

        void applyStates() {
            switch (systemState) {
                case SystemState::CLOSED_UNLOADED:
                    if (globals::piston.is_extended()) globals::piston.retract();
                    break;
                case SystemState::CLOSED_LOADED:
                    if (globals::piston.is_extended()) globals::piston.retract();
                    break;
                case SystemState::OPEN:
                    if (!globals::piston.is_extended()) globals::piston.extend();
                    break;
            }
        }
    }

    void initialize() {

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
}