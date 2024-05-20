#ifndef DMPLIB_INTEGRABLE_CLASS_HPP__
#define DMPLIB_INTEGRABLE_CLASS_HPP__

#include <chrono>

#include "dmplib/utils/macros.hpp"

namespace dmp {

template <typename Derived>
class Integrable {
protected:
    double _dt;

public:
    Integrable() { _dt = 0.001; }

    Integrable(const double dt) { _dt = dt; };

    void
    step() {
        static_cast<Derived*>(this)->step_impl();
    }

    GET_SET(_dt, integration_period);

    template <typename Rep, typename Period>
    void set_integration_period(const std::chrono::duration<Rep, Period>& dt);

    void
    set_integration_frequency(const double& freq_hz) {
        _dt = 1 / freq_hz;
    }
};

template <typename D>
template <typename Rep, typename Period>
void
Integrable<D>::set_integration_period(const std::chrono::duration<Rep, Period>& dt) {
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;
    set_integration_period(duration_cast<nanoseconds>(dt).count() * 1e-9);
}

}  // namespace dmp

#endif  // DMPLIB_INTEGRABLE_CLASS_HPP__
