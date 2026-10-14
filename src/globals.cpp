#include "globals.hpp"

namespace globals {
    // Drivetrain
    ez::Drive chassis({-12, -1}, {10, 19}, 2, 4.125, 200.0);
    pros::Imu& imu = chassis.imu;

    // Lift
    pros::MotorGroup liftMotors({11, 20}, pros::MotorGears::green, pros::MotorUnits::rotations);

    // Intake
    pros::adi::Pneumatics piston('A', false);
    pros::Distance distance(21);
}
