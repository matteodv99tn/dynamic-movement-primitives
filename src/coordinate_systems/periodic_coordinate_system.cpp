#include "dmplib/coordinate_systems/periodic_coordinate_system.hpp"

#include <cmath>

#include "dmplib/coordinate_systems/coordinate_system.hpp"

using Pcs = dmp::PeriodicCs;

Pcs::PeriodicCs() : CoordinateSystem<Pcs, PERIODIC>::CoordinateSystem(0.0) {}

void
Pcs::step_impl() {
    _x += 2 * M_PI / _T * _dt;
}

double
Pcs::get_Omega() const {
    return 1 / _T;
}
