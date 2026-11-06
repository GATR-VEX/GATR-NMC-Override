#include "subsystems/intake.hpp"
#include "utilities/logger.hpp"

#include <cstdint>

using utilities::Logger;

namespace intake {
    namespace {
        namespace k = constants::intake;

        WantedState wanted_state = WantedState::CLOSED;
        SystemState system_state = SystemState::CLOSED_UNLOADED;

        int cycle_count = 0;

        std::int32_t distance_mm = 0;
        bool sensor_ok = false;
        bool piece_present = false;
        std::uint32_t present_since_ms = 0;

        bool armed = false;

        void update_sensor() {
            sensor_ok = globals::distance.is_installed();
            distance_mm = sensor_ok ? globals::distance.get_distance() : 0;

            if (!sensor_ok || distance_mm == PROS_ERR || distance_mm < 0) {
                sensor_ok = false;
                piece_present = false;
                return;
            }

            if (!piece_present && distance_mm <= k::GAME_PIECE_DETECTION_DISTANCE_MM) {
                piece_present = true;
                present_since_ms = pros::millis();
            } else if (piece_present && distance_mm >= k::GAME_PIECE_CLEAR_DISTANCE_MM) {
                piece_present = false;
            }
        }

        bool piece_confirmed() {
            return piece_present && pros::millis() - present_since_ms >= k::PIECE_DETECTION_TIME_MS;
        }

        SystemState handle_state_transition() {
            switch (wanted_state) {
                case WantedState::CLOSED:
                    return piece_present ? SystemState::CLOSED_LOADED : SystemState::CLOSED_UNLOADED;

                case WantedState::OPEN:
                    return SystemState::OPENED;

                case WantedState::AUTO_INTAKE:
                    if (!piece_present) {
                        armed = true;
                    }

                    if (armed && piece_confirmed()) {
                        wanted_state = WantedState::CLOSED;
                        return SystemState::CLOSED_LOADED;
                    }

                    return SystemState::WAITING_FOR_PIECE;
            }

            return SystemState::CLOSED_UNLOADED;
        }

        void apply_states() {
            switch (system_state) {
                case SystemState::CLOSED_UNLOADED:
                case SystemState::CLOSED_LOADED:
                    if (globals::piston.is_extended()) globals::piston.retract();
                    break;

                case SystemState::OPENED:
                case SystemState::WAITING_FOR_PIECE:
                    if (!globals::piston.is_extended()) globals::piston.extend();
                    break;
            }
        }

        bool is_open(SystemState state) {
            return state == SystemState::OPENED || state == SystemState::WAITING_FOR_PIECE;
        }

        const char* wanted_state_name(WantedState state) {
            switch (state) {
                case WantedState::OPEN: return "OPEN";
                case WantedState::CLOSED: return "CLOSED";
                case WantedState::AUTO_INTAKE: return "AUTO_INTAKE";
            }

            return "?";
        }

        const char* system_state_name(SystemState state) {
            switch (state) {
                case SystemState::OPENED: return "OPENED";
                case SystemState::CLOSED_UNLOADED: return "CLOSED_UNLOADED";
                case SystemState::CLOSED_LOADED: return "CLOSED_LOADED";
                case SystemState::WAITING_FOR_PIECE: return "WAITING_FOR_PIECE";
            }

            return "?";
        }

        void log_outputs() {
            Logger::recordOutput("Intake/WantedState", static_cast<int>(wanted_state));
            Logger::recordOutput("Intake/SystemState", static_cast<int>(system_state));
            Logger::recordOutput("Intake/CycleCount", cycle_count);
            Logger::recordOutput("Intake/DistanceMillimeters", sensor_ok ? distance_mm : -1);
            Logger::recordOutput("Intake/PiecePresent", piece_present);
            Logger::recordOutput("Intake/Armed", armed);
            Logger::recordOutput("Intake/PistonExtended", globals::piston.is_extended());
        }
    }

    void initialize() {
        const char* outputs[] = {
            "Intake/WantedState",
            "Intake/SystemState",
            "Intake/CycleCount",
            "Intake/DistanceMillimeters",
            "Intake/PiecePresent",
            "Intake/Armed",
            "Intake/PistonExtended",
        };

        for (const char* output : outputs) {
            Logger::registerOutput(output);
        }
    }

    void periodic() {
        update_sensor();

        const SystemState next_state = handle_state_transition();

        if (is_open(next_state) && !is_open(system_state)) {
            ++cycle_count;
        }

        system_state = next_state;
        apply_states();
        log_outputs();
    }

    void set_wanted_state(WantedState state) {
        if (wanted_state == state) return;

        if (state == WantedState::AUTO_INTAKE) {
            armed = false;
        }

        wanted_state = state;
    }

    WantedState get_wanted_state() {
        return wanted_state;
    }

    SystemState get_system_state() {
        return system_state;
    }

    bool has_piece() {
        return piece_present;
    }

    void screen_subtask() {
        ez::screen_print(std::string("Wanted State: ") + wanted_state_name(wanted_state), 1);
        ez::screen_print(std::string("System State: ") + system_state_name(system_state), 2);
        ez::screen_print("Distance: " + (sensor_ok ? std::to_string(distance_mm) + " mm" : std::string("NOT CONNECTED")) + (piece_present ? "  PIECE" : ""), 3);
        ez::screen_print(std::string("Piston: ") + (globals::piston.is_extended() ? "EXTENDED" : "RETRACTED") + (armed ? "  armed" : ""), 4);
        ez::screen_print("Cycle Count: " + util::to_string_with_precision(cycle_count), 5);
    }
}