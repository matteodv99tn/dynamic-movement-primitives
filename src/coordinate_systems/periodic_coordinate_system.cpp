#include "dmplib/coordinate_systems/periodic_coordinate_system.hpp"

#include <cmath>

#include "dmplib/coordinate_systems/coordinate_system.hpp"

using Pcs_t = dmp::PeriodicCs;

Pcs_t::PeriodicCs(std::reference_wrapper<const double> T) :
        CoordinateSystem<Pcs_t, PERIODIC>::CoordinateSystem(T, 0.0) {
}

void
Pcs_t::step_impl() {
    _x += 2 * M_PI / _T * _dt;
}

double
Pcs_t::get_Omega() const {
    return 1 / _T;
}

double
Pcs_t::compute_coord_impl(const double& time) const {
    return 2 * M_PI / _T * time;
}
