#pragma once

#include <algorithm>
#include <cmath>

namespace utilities {
    class ArmFeedforward {
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
            ArmFeedforward(double kS, double kV, double kG, double kA = 0.0, double period = 0.01): kS(kS), kV(std::max(0.0, kV)), kG(kG), kA(std::max(0.0, kA)), period(period > 0.0 ? period : 0.01) {}

            double calculate(double angle_radians, double velocity) const {
                return kS * sign(velocity) + kV * velocity + kG * std::cos(angle_radians);
            }

            double calculate_with_velocities(double angle_radians, double current_velocity, double next_velocity) const {
                const double acceleration = (next_velocity - current_velocity) / period;
                return kS * sign(current_velocity) + kV * current_velocity + kG * std::cos(angle_radians) + kA * acceleration;
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