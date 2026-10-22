#pragma once

#include <algorithm>
#include <cmath>

namespace utilities {
    class ElevatorFeedforward {
        private:
            double kS;
            double kV;
            double kG;
            double kA;
            double period;

            static double sign(double value) {
                return (value > 0.0) - (value < 0.0);
            }

        public:
            ElevatorFeedforward(double kS, double kV, double kG, double kA = 0.0, double perioid = 0.01): kS(kS), kV(std::max(0.0, kV)), kG(kG), kA(std::max(0.0, kA)), period(perioid > 0.0 ? period : 0.01) {}
            
            double calculate(double velocity) const {
                return calculate_with_velocities(velocity, velocity);
            }

            double calculate_with_velocities(double current_velocity, double next_velocity) const {
                if (kA < 1e-9) {
                    return kS * sign(next_velocity) + kG + kV * next_velocity;
                }

                const double A = -kV / kA;
                const double ad = std::exp(A * period);
                const double bd = kV < 1e-9 ? period / kA : -std::expm1(A * period) / kV;

                return kG + kS * sign(current_velocity) + (next_velocity - ad * current_velocity) / bd;
            }

            void set_ks(double value) {
                kS = value;
            }

            void set_kv(double value) {
                kV = value;
            }

            void set_kg(double value) {
                kG = value;
            }

            void set_ka(double value) {
                kA = value;
            }

            double get_ks() const {
                return kS;
            }

            double get_kv() const {
                return kV;
            }

            double get_kg() const {
                return kG;
            }

            double get_ka() const {
                return kA;
            }

            double get_period() const {
                return period;
            }
    };
}