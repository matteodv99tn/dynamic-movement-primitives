#include "dmplib/coordinate_systems/periodic_coordinate_system.hpp"

#include <cmath>

using Pcs = dmp::PeriodicCs;

Pcs::PeriodicCs() : CoordinateSystem(0) {}

void
Pcs::step() {
    _x += 2 * M_PI / _T * _dt;
}

double
Pcs::get_Omega() const {
    return 1 / _T;
}
