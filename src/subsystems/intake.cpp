#include "subsystems/intake.hpp"

namespace intake {
    namespace {
        WantedState wanted_state = WantedState::CLOSED;
        SystemState system_state = SystemState::CLOSED_UNLOADED;

        int cycle_cout = 0;

        SystemState handle_state_transition() {
            switch (wanted_state) {
                case WantedState::CLOSED:
                    return SystemState::CLOSED_LOADED;
                case WantedState::OPEN:
                    if(system_state != SystemState::OPENED) {
                        ++cycle_cout;
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

		std::string wanted_state_to_string(WantedState state) {
		    switch (state) {
				case WantedState::CLOSED:
					return "CLOSED";
				case WantedState::OPEN:
					return "OPEN";
			}

			return "INDETERMINITE";
		}

		std::string system_state_to_string(SystemState state) {
			switch (state) {
				case SystemState::CLOSED_UNLOADED:
					return "CLOSED_UNLOADED";
				case SystemState::CLOSED_LOADED:
					return "CLOSED_LOADED";
				case SystemState::OPENED:
					return "OPENED";
			}

			return "INDETERMINITE";
		}
    }

    void initialize() {

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

    void screen_subtask() {
		ez::screen_print("Wanted State: " + wanted_state_to_string(wanted_state), 1);
		ez::screen_print("System State: " + system_state_to_string(system_state), 2);
        ez::screen_print("Piston State: " + globals::piston.is_extended(), 4);
        ez::screen_print("Cycle Count: " + util::to_string_with_precision(cycle_cout), 5);
    }
}
