#include "dmplib/coordinate_systems/exponential_decay_cs.hpp"

using Edcs_t = dmp::ExponentialDecayCs;

Edcs_t::ExponentialDecayCs(const double& alpha) : _alpha(alpha) {
}

void
Edcs_t::step_impl() {
    _x -= _alpha * _x * _dt;
}
