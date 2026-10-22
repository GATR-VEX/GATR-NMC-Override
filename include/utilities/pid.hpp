#pragma once

#include <algorithm>
#include <cmath>

namespace utilities {
    class PIDController {
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
        public:
            PIDController(double kP, double kI, double kD): kP(kP), kI(kI), kD(kD) {}
            
            double calculate(double measurement, double setpoint, double dt) {
                if (!std::isfinite(measurement) || !std::isfinite(setpoint) || !std::isfinite(dt) || dt <= 0.0) {
                    reset();
                    return 0.0;
                }

                error = setpoint - measurement;
                error_derivative = 0.0;

                if (has_previous_measurement) {
                    error_derivative = -(measurement - previous_measurement) / dt;
                }

                previous_measurement = measurement;
                has_previous_measurement = true;

                if (std::abs(error) > i_zone) {
                    integral = 0.0;
                } else if (kI != 0.0) {
                    integral += error * dt;

                    if (kI > 0.0) {
                        integral = std::clamp(integral, minimum_integral / kI, maximum_integral / kI);
                    }
                }

                return kP * error + kI * integral + kD * error_derivative;
            }

            void reset() {
                integral = 0.0;
                error = 0.0;
                error_derivative = 0.0;

                previous_measurement = 0.0;
                has_previous_measurement = false;
            }

            void set_pid(double p, double i, double d) {
                kP = p,
                kI = i;
                kD = d;
            }

            void set_i_zone(double zone) {
                if (zone >= 0.0) {
                    i_zone = zone;
                }
            }

            void set_integral_range(double minimum, double maximum) {
                if (minimum <= maximum) {
                    minimum_integral = minimum;
                    maximum_integral = maximum;
                }
            }

            void set_tolerence(double position, double velocity = std::numeric_limits<double>::infinity()) {
                if (position >= 0.0 && velocity >= 0.0) {
                    position_tolerence = position;
                    velocity_tolerence = velocity;
                }
            }

            bool at_setpoint() const {
                return has_previous_measurement && std::abs(error) < position_tolerence && std::abs(error_derivative) < velocity_tolerence;
            }

            double get_error() const {
                return error;
            }

            double get_error_derivative() const {
                return error_derivative;
            }
    };
}