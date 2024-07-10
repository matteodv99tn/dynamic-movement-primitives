#include "dmplib/coordinate_systems/periodic_coordinate_system.hpp"

#include <cmath>

#include "dmplib/coordinate_systems/coordinate_system.hpp"
#include "dmplib/time_axis.hpp"

using Pcs_t = dmp::PeriodicCs;

Pcs_t::PeriodicCs(dmp::TimeAxis::Reference time_axis) :
        CoordinateSystem<Pcs_t, PERIODIC>::CoordinateSystem(time_axis, 0.0) {
}

void
Pcs_t::step_impl() {
    _x += 2 * M_PI / T() * dt();
}

double
Pcs_t::get_Omega() const {
    return 1 / T();
}

double
Pcs_t::compute_coord_impl(const double& time) const {
    return 2 * M_PI / T() * time;
}
