#pragma once

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

            explicit MotionProfiler(Constraints constraints);
            State calculate(double dt, State current, State goal) const;

        private:
            Constraints constraints_;
    };
}