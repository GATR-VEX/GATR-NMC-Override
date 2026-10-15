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
            double previousMeasurement = 0.0;
            bool hasPreviousMeasurement = false;

            double minimumIntegral = -1e6;
            double maximumIntegral = 1e6;
        public:
            PIDController(double kP, double kI, double kD): kP(kP), kI(kI), kD(kD) {}
            
            double calculate(double measurement, double setpoint, double dt) {
                const double error = setpoint - measurement;

                if (dt <= 0 || !std::isfinite(dt)) {
                    return kP * error;
                }

                integral += error * dt;
                integral = std::clamp(integral, minimumIntegral, maximumIntegral);

                double derivative = 0.0;

                if (hasPreviousMeasurement) {
                    derivative = -(measurement - previousMeasurement) / dt;
                }

                previousMeasurement = measurement;
                hasPreviousMeasurement = true;

                return kP * error + kI * integral + kD * derivative;
            }

            void reset() {
                integral = 0.0;
                previousMeasurement = 0.0;
                hasPreviousMeasurement = false;
            }

            void setIntegralRange(double minimum, double maximum) {
                minimumIntegral = minimum;
                maximumIntegral = maximum;
            }
    };
}