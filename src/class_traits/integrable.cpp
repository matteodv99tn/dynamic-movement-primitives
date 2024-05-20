#include "dmplib/class_traits/integrable.hpp"

#include <chrono>

namespace dmp {
Integrable::Integrable() { _dt = 0.001; }

Integrable::Integrable(const double dt) { _dt = dt; }

void
Integrable::set_integration_period(double dt) {
    this->_dt = dt;
}

double
Integrable::get_integration_period() const {
    return _dt;
}

void
Integrable::set_integration_frequency(const double& freq_hz) {
    _dt = 1 / freq_hz;
}

}  // namespace dmp
