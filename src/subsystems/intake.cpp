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
            const bool shouldExtend = systemState == SystemState::OPEN;

            if (globals::piston.is_extended() != shouldExtend) {
                if (shouldExtend) {
                    globals::piston.extend();
                } else {
                    globals::piston.retract();
                }
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