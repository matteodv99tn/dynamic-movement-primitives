#ifndef DMPLIB_INTEGRABLE_CLASS_HXX__
#define DMPLIB_INTEGRABLE_CLASS_HXX__

#include <chrono>

#include "dmplib/class_traits/integrable.hpp"

namespace dmp {
Integrable::Integrable() { _dt = 0.001; }

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

#endif  // DMPLIB_INTEGRABLE_CLASS_HXX__
