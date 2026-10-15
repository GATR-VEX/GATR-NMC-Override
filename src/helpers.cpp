#include "helpers.hpp"

namespace helpers {
    double wrapHeading(double degrees) {
        return std::remainder(degrees, 360.0);
    }

    double rotationsToRadians(double rotations) {
        return 2 * rotations * constants::kPi;
    }

    double radiansToRotations(double radians) {
        return radians / (2 * constants::kPi);
    }
}