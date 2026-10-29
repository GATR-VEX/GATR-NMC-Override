
#pragma once

#include <algorithm>
#include <cmath>

namespace controls {
    class SimpleMotorFeedforward {
        private:
            double kS;
            double kV;
            double kA;
            double period;

            static double sign(double value) {
                return (value > 0.0) - (value < 0.0);
            }
        public:
            SimpleMotorFeedforward(double kS, double kV, double kA = 0.0, double period = 0.01) : kS(kS), kV(std::max(0.0, kV)), kA(std::max(0.0, kA)), period(period > 0.0 ? period : 0.02) {}

            double calculate(double velocity) const {
                return calculate_with_velocities(velocity, velocity);
            }

            double calculate_with_velocities(double current_velocity, double next_velocity) const {
                if (kA < 1e-9) {
                    return kS * sign(next_velocity) + kV * next_velocity;
                }

                const double A = -kV / kA;
                const double Ad = std::exp(A * period);

                const double Bd = kV < 1e-9 ? period / kA : -std::expm1(A * period) / kV;

                return kS * sign(current_velocity) + (next_velocity - Ad * current_velocity) / Bd;
            }

            void set_ks(double value) { 
                kS = value; 
            }

            void set_kv(double value) { 
                if (value >= 0.0) kV = value; 
            }

            void set_ka(double value) { 
                if (value >= 0.0) kA = value; 
            }

            double get_ks() const { 
                return kS; 
            }

            double get_kv() const { 
                return kV; 
            }

            double get_ka() const { 
                return kA; 
            }

            double get_period() const { 
                return period; 
            }
    };
}