#pragma once

#include <limits>

namespace controls {
    class PIDController {
        public:
            PIDController(double kP, double kI, double kD);

            double calculate(double measurement, double setpoint, double dt);
            void reset();

            void set_pid(double p, double i, double d);
            void set_i_zone(double zone);
            void set_integral_range(double minimum, double maximum);
            void set_tolerence(double position, double velocity = std::numeric_limits<double>::infinity());

            bool at_setpoint() const;
            double get_error() const;
            double get_error_derivative() const;
        private:
            double kP;
            double kI;
            double kD;

            double integral = 0.0;
            double previous_measurement = 0.0;
            bool has_previous_measurement = false;

            double error = 0.0;
            double error_derivative = 0.0;

            double minimum_integral = -1e6;
            double maximum_integral = 1e6;

            double i_zone = std::numeric_limits<double>::infinity();

            double position_tolerence = 0.05;
            double velocity_tolerence = std::numeric_limits<double>::infinity();
    };
}