#include "subsystems/intake.hpp"
#include "utilities/logger.hpp"

using utilities::Logger;

namespace intake {
    namespace {
        WantedState wanted_state = WantedState::CLOSED;
        SystemState system_state = SystemState::CLOSED_UNLOADED;

        int cycle_count = 0;

        bool is_distance_connected() {
            return globals::distance.is_installed();
        }

        SystemState handle_state_transition() {
            switch (wanted_state) {
                case WantedState::CLOSED:
                    if (is_distance_connected() && globals::distance.get_distance() < 15) {
                        return SystemState::CLOSED_LOADED;
                    }

                    return SystemState::CLOSED_UNLOADED;
                case WantedState::OPEN:
                    if(system_state != SystemState::OPENED) {
                        ++cycle_count;
                    }

                    return SystemState::OPENED;
            }

            return SystemState::CLOSED_UNLOADED;
        }

        void apply_states() {
            switch (system_state) {
                case SystemState::CLOSED_UNLOADED:
                    if (globals::piston.is_extended()) globals::piston.retract();
                    break;
                case SystemState::CLOSED_LOADED:
                    if (globals::piston.is_extended()) globals::piston.retract();
                    break;
                case SystemState::OPENED:
                    if (!globals::piston.is_extended()) globals::piston.extend();
                    break;
            }
        }

		const char* wanted_state_name(WantedState state) {
            switch (state) {
                case WantedState::OPEN: return "OPEN";
                case WantedState::CLOSED: return "CLOSED";
            }

            return "?";
        }

        const char* system_state_name(SystemState state) {
            switch (state) {
                case SystemState::OPENED: return "OPENED";
                case SystemState::CLOSED_UNLOADED: return "CLOSED_UNLOADED";
                case SystemState::CLOSED_LOADED: return "CLOSED_LOADED";
            }

            return "?";
        }

        void log_outputs() {
            Logger::recordOutput("Intake/WantedState", static_cast<int>(wanted_state));
            Logger::recordOutput("Intake/SystemState", static_cast<int>(system_state));
            Logger::recordOutput("Intake/CycleCount", cycle_count);
            Logger::recordOutput("Intake/DistanceMillimeters", is_distance_connected() ? globals::distance.get_distance() : 0.0);
            Logger::recordOutput("Intake/PistionExtended", globals::piston.is_extended());
            Logger::recordOutput("Intake/IsLoaded", (system_state == SystemState::CLOSED_LOADED));
        }
    }

    void initialize() {
        const char* outputs[] = {
            "Intake/WantedState",
            "Intake/SystemState",
            "Intake/CycleCount",
            "Intake/DistanceMillimeters",
            "Intake/PistionExtended",
            "Intake/IsLoaded"
        };

        for (const char* output : outputs) {
            Logger::registerOutput(output);
        }
    }

    void periodic() {
        log_outputs();

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

    void screen_subtask() {
		ez::screen_print("Wanted State: " + static_cast<int>(wanted_state), 1);
		ez::screen_print("System State: " + static_cast<int>(system_state), 2);

        ez::screen_print("Piston State: " + globals::piston.is_extended(), 4);
        ez::screen_print("Cycle Count: " + util::to_string_with_precision(cycle_count), 5);
    }
}
