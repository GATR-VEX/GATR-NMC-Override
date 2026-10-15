#pragma once

#include <cmath>

namespace utilities {
    inline double sign(double value) {
        return (value > 0.0) - (value < 0.0);
    }

    class SimpleMotorFeedforward {
        public:
            SimpleMotorFeedforward(double kS, double kV, double kA): kS(kS), kV(kV), kA(kA) {};

            double calculate(double velocity, double acceleration = 0.0, double direction = 0.0) const {
                const double direction = velocity != 0.0 ? velocity : direction;
                return kS * sign(direction) + kV * velocity + kA * acceleration;
            }
        private:
            double kS;
            double kV;
            double kA;
    };

    class ElevatorFeedforward {
        private:
            SimpleMotorFeedforward feedforward;
            double kG;
        public: 
            ElevatorFeedforward(double kS, double kV, double kG, double kA): feedforward(kS, kV, kA), kG(kG) {};

            double calculate(double velocity = 0.0, double acceleration = 0.0, double direction = 0.0) const {
                return kG + feedforward.calculate(velocity, acceleration, direction);
            }
    };

    class ArmFeedforward {
        private:
            SimpleMotorFeedforward feedforward;
            double kG;
        public:
            ArmFeedforward(double kS, double kV, double kG, double kA): feedforward(kS, kV, kA), kG(kG) {};

            double calculate(double angle, double velocity = 0.0, double acceleration = 0.0, double direction = 0.0) const {
                return std::cos(angle) * kG + feedforward.calculate(velocity, acceleration, direction);
            }
    };
}