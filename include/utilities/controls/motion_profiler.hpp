#pragma once

#include <algorithm>
#include <cmath>

namespace controls {
    class MotionProfiler {
        public:
            struct Constraints {
                double max_velocity;
                double max_acceleration;
            };

            struct State {
                double position = 0.0;
                double velocity = 0.0;
            };

            explicit MotionProfiler(Constraints constraints) : constraints_(constraints) {}

            State calculate(double dt, State current, State goal) const {
                const double max_vel = constraints_.max_velocity;
                const double max_accel = constraints_.max_acceleration;

                if (
                    dt <= 0.0 ||
                    !std::isfinite(dt) ||
                    max_vel <= 0.0 ||
                    max_accel <= 0.0 ||
                    !std::isfinite(max_vel) ||
                    !std::isfinite(max_accel) ||
                    !std::isfinite(current.position) ||
                    !std::isfinite(current.velocity) ||
                    !std::isfinite(goal.position) ||
                    !std::isfinite(goal.velocity)
                ) {
                    return current;
                }

                State result = current;

                goal.velocity = std::clamp(goal.velocity, -max_vel, max_vel);
                double recovery_time = 0.0;

                if (std::abs(current.velocity) > max_vel) {
                    recovery_time = (std::abs(current.velocity) - max_vel) / max_accel;

                    const double recovery_acceleration = -std::copysign(max_accel, current.velocity);
                    current.position += current.velocity * recovery_time + 0.5 * recovery_acceleration * recovery_time * recovery_time;
                    current.velocity = std::copysign(max_vel, current.velocity);
                }

                const double distance = goal.position - current.position;
                const double stoppingDistance = std::abs(goal.velocity - current.velocity) * (goal.velocity + current.velocity) / (2.0 * max_accel);
                const double direction = std::abs(distance - stoppingDistance) < 1e-12 ? std::copysign(1.0, goal.velocity) : (distance > stoppingDistance ? 1.0 : -1.0);
                const double acceleration = direction * max_accel;
                const double velocityLimit = direction * max_vel;
                const double peak_velocity_squared = acceleration * distance + 0.5 * (goal.velocity * goal.velocity + current.velocity * current.velocity);
                const double peak_velocity = direction * std::sqrt(std::max(0.0, peak_velocity_squared));

                double acceleration_time = 0.0;
                double cruise_time = 0.0;
                double deceleration_time = 0.0;

                if (direction * peak_velocity > max_vel) {
                    acceleration_time = (velocityLimit - current.velocity) / acceleration;
                    deceleration_time = (velocityLimit - goal.velocity) / acceleration;

                    const double accelerationDistance = (velocityLimit * velocityLimit - current.velocity * current.velocity) / (2.0 * acceleration);
                    const double decelerationDistance = (velocityLimit * velocityLimit - goal.velocity * goal.velocity) / (2.0 * acceleration);

                    cruise_time = (distance - accelerationDistance - decelerationDistance) / velocityLimit;
                } else {
                    acceleration_time = (peak_velocity - current.velocity) / acceleration;
                    deceleration_time = (peak_velocity - goal.velocity) / acceleration;
                }

                acceleration_time += recovery_time;

                auto advance = [](State& state, double time, double accel) {
                    state.position += state.velocity * time + 0.5 * accel * time * time;
                    state.velocity += accel * time;
                };

                advance(result, std::min(dt, acceleration_time), recovery_time > 0.0 && result.velocity * direction > 0.0 ? -acceleration : acceleration);

                if (dt <= acceleration_time) return result;

                dt -= acceleration_time;
    
                advance(result, std::min(dt, cruise_time), 0.0);

                if (dt <= cruise_time) return result;
                

                dt -= cruise_time;

                advance(result, std::min(dt, deceleration_time), -acceleration);

                if (dt > deceleration_time) return goal;
                return result;
            }

        private:
            Constraints constraints_;
    };
}