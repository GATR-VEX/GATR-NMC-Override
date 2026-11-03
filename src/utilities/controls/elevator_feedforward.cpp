#include "utilities/controls/elevator_feedforward.hpp"

#include <algorithm>
#include <cmath>

namespace controls {
    namespace {
        double sign(double value) {
            return (value > 0.0) - (value < 0.0);
        }
    }

    ElevatorFeedforward::ElevatorFeedforward(double kS, double kV, double kG, double kA, double period): kS(kS), kV(std::max(0.0, kV)), kG(kG), kA(std::max(0.0, kA)), period(period > 0.0 ? period : 0.01) {}

    double ElevatorFeedforward::calculate(double velocity) const {
        return calculate_with_velocities(velocity, velocity);
    }

    double ElevatorFeedforward::calculate_with_velocities(double current_velocity, double next_velocity) const {
        if (kA < 1e-9) {
            return kS * sign(next_velocity) + kG + kV * next_velocity;
        }

        const double A = -kV / kA;
        const double ad = std::exp(A * period);
        const double bd = kV < 1e-9 ? period / kA : -std::expm1(A * period) / kV;

        return kG + kS * sign(current_velocity) + (next_velocity - ad * current_velocity) / bd;
    }

    void ElevatorFeedforward::set_ks(double value) { 
        kS = value;
    }

    void ElevatorFeedforward::set_kv(double value) { 
        kV = value; 
    }

    void ElevatorFeedforward::set_kg(double value) { 
        kG = value; 
    }

    void ElevatorFeedforward::set_ka(double value) { 
        kA = value; 
    }

    double ElevatorFeedforward::get_ks() const { 
        return kS; 
    }

    double ElevatorFeedforward::get_kv() const { 
        return kV; 
    }

    double ElevatorFeedforward::get_kg() const { 
        return kG; 
    }

    double ElevatorFeedforward::get_ka() const { 
        return kA; 
    }

    double ElevatorFeedforward::get_period() const { 
        return period; 
    }
}