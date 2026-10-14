#include "helper.h"

double wrapHeading(double degrees) {
    return std::remainder(degrees, 360.0);
}