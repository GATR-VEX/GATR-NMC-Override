#include "utilities/controls/arm_feedforward.hpp"

#include <algorithm>
#include <cmath>

namespace controls {
    namespace {
        double sign(double value) {
            return (value > 0.0) - (value < 0.0);
        }
    }

    ArmFeedforward::ArmFeedforward(double kS, double kV, double kG, double kA, double period): kS(kS), kV(std::max(0.0, kV)), kG(kG), kA(std::max(0.0, kA)), period(period > 0.0 ? period : 0.01) {}

    double ArmFeedforward::calculate(double angle_radians, double velocity, double acceleration) const {
        return kS * sign(velocity) + kV * velocity + kG * std::cos(angle_radians) + kA * acceleration;
    }

    double ArmFeedforward::calculate(double angle_radians, double velocity) const {
        return kS * sign(velocity) + kV * velocity + kG * std::cos(angle_radians);
    }

    double ArmFeedforward::calculate_with_velocities(double angle_radians, double current_velocity, double next_velocity) const {
        const double acceleration = (next_velocity - current_velocity) / period;
        return kS * sign(current_velocity) + kV * current_velocity + kG * std::cos(angle_radians) + kA * acceleration;
    }

    void ArmFeedforward::set_ks(double value) {
        kS = value; 
    }

    void ArmFeedforward::set_kv(double value) { 
        kV = value; 
    }

    void ArmFeedforward::set_kg(double value) { 
        kG = value;
    }

    void ArmFeedforward::set_ka(double value) { 
        kA = value; 
    }

    double ArmFeedforward::get_ks() const { 
        return kS; 
    }

    double ArmFeedforward::get_kv() const { 
        return kV; 
    }

    double ArmFeedforward::get_kg() const { 
        return kG; 
    }
    
    double ArmFeedforward::get_ka() const { 
        return kA; 
    }

    double ArmFeedforward::get_period() const { 
        return period; 
    }
}