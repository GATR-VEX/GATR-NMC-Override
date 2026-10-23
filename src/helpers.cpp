#include "helpers.hpp"

namespace helpers {
    double wrap_heading(double degrees) {
        return std::remainder(degrees, 360.0);
    }

    double rotations_to_radians(double rotations) {
        return 2 * rotations * constants::PI;
    }

    double radians_to_rotations(double radians) {
        return radians / (2 * constants::PI);
    }
}