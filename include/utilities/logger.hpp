#pragma once

#include <cstddef>

namespace utilities {
	class Logger {
		public:
			static constexpr std::size_t MAX_OUTPUTS = 48;

			static void initialize();

			static bool registerOutput(const char* name);
			static bool recordOutput(const char* name, double value);
			static bool getOutput(const char* name, double& value);
			static bool start(const char* sessionName = "practice");
			static bool stop();

			Logger() = delete;
	};
}
