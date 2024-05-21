#ifndef DMPLIB_INTEGRABLE_CLASS_HXX__
#define DMPLIB_INTEGRABLE_CLASS_HXX__

#include <chrono>

#include "dmplib/class_traits/integrable.hpp"

namespace dmp {

template <typename D>
Integrable<D>::Integrable(const double& dt) {
    _dt = dt;
}

template <typename D>
void
Integrable<D>::step() {
    static_cast<D*>(this)->step_impl();
}

template <typename D>
void
Integrable<D>::set_integration_period(const double& dt) {
    this->_dt = dt;
}

template <typename D>
template <typename Rep, typename Period>
void
Integrable<D>::set_integration_period(const std::chrono::duration<Rep, Period>& dt) {
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;
    set_integration_period(duration_cast<nanoseconds>(dt).count() * 1e-9);
}

template <typename D>
void
Integrable<D>::set_integration_frequency(const double& freq_hz) {
    _dt = 1 / freq_hz;
}

}  // namespace dmp

#endif  // DMPLIB_INTEGRABLE_CLASS_HXX__
