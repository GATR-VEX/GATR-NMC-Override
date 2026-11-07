#include "utilities/robot_status.hpp"
#include "utilities/logger.hpp"
#include "api.h"

using utilities::Logger;

namespace robot_status {
	namespace {
		double checked(double value) {
			return std::isfinite(value) ? value : 0.0;
		}

		double checked(std::int32_t value) {
			return value == PROS_ERR ? -1 : static_cast<double>(value);
		}
	}

	void initialize() {
		const char* outputs[] = {
			"Battery/CapacityPercent",
            "Battery/CurrentMilliamps",
            "Battery/TemperatureCelsius",
            "Battery/Volts",
            "Competition/Status",
            "Competition/Autonomous",
            "Competition/Connected",
            "Competition/FieldControl",
            "Competition/Disabled"
		};

		for (const char* output : outputs) {
			Logger::registerOutput(output);
		}
	}

	void periodic() {
        Logger::recordOutput("Battery/CapacityPercent", checked(pros::battery::get_capacity()));
        Logger::recordOutput("Battery/CurrentMilliamps", checked(pros::battery::get_current()));
        Logger::recordOutput("Battery/TemperatureCelsius", checked(pros::battery::get_temperature()));

        const std::int32_t millivolts = pros::battery::get_voltage();
        Logger::recordOutput("Battery/Voltage", millivolts == PROS_ERR ? -1.0 : millivolts / 1000.0);

        const std::uint8_t status = pros::competition::get_status();
        Logger::recordOutput("Competition/Status", status);
        Logger::recordOutput("Competition/Autonomous", (status & COMPETITION_AUTONOMOUS) != 0);
        Logger::recordOutput("Competition/Connected", (status & COMPETITION_CONNECTED) != 0);
        Logger::recordOutput("Competition/FieldControl", (status & COMPETITION_SYSTEM) != 0);
        Logger::recordOutput("Competition/Disabled", (status & COMPETITION_DISABLED) != 0);
    }
}
