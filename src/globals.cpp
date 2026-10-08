#include "globals.hpp"

namespace globals {
    ez::Drive chassis(
        {-12, -14},
        {11, 13},
        1,
        4.125,
        200.0
    );

    pros::Imu& imu = chassis.imu;
}
