#include "globals.hpp"

namespace globals {
    // Drivetrain
    ez::Drive chassis({-12, -1}, {10, 19}, 2, 4.125, 200.0);
    pros::Imu& imu = chassis.imu;

    // Lift
    pros::MotorGroup lift_motors({11, -20}, pros::MotorGears::red, pros::MotorUnits::rotations);

    // Intake
    pros::adi::Pneumatics piston('A', false);
    pros::Distance distance(21);

    // Controller
    utilities::EventLoop bindings;
    utilities::Controller primary(pros::E_CONTROLLER_MASTER, bindings);
    utilities::Controller secondary(pros::E_CONTROLLER_PARTNER, bindings);
}
