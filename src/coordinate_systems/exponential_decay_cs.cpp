#include "dmplib/coordinate_systems/exponential_decay_cs.hpp"

using EdCs = dmp::ExponentialDecayCs;

EdCs::ExponentialDecayCs(const double& alpha) {
    _alpha = alpha;
}

void EdCs::step_impl() {
    _x -= _alpha * _x * _dt;
}
