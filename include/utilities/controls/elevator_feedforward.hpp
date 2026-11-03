#pragma once

namespace controls {
    class ElevatorFeedforward {
        public:
            ElevatorFeedforward(double kS, double kV, double kG, double kA = 0.0, double period = 0.01);

            double calculate(double velocity) const;
            double calculate_with_velocities(double current_velocity, double next_velocity) const;

            void set_ks(double value);
            void set_kv(double value);
            void set_kg(double value);
            void set_ka(double value);

            double get_ks() const;
            double get_kv() const;
            double get_kg() const;
            double get_ka() const;
            double get_period() const;
        private:
            double kS;
            double kV;
            double kG;
            double kA;
            double period;
    };
}