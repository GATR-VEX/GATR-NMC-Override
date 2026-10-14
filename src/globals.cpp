#include "globals.hpp"

namespace globals {
    ez::Drive chassis(
        {-12, -1},
        {10, 19},
        2,
        4.125,
        200.0
    );

    pros::Imu& imu = chassis.imu;
    pros::adi::Pneumatics piston('A', false);
}
