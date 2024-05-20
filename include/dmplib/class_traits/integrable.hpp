#ifndef DMPLIB_INTEGRABLE_CLASS_HPP__
#define DMPLIB_INTEGRABLE_CLASS_HPP__

#include <chrono>

#include "dmplib/utils/macros.hpp"

namespace dmp {
class Integrable {
protected:
    double _dt;

public:
    Integrable();

    Integrable(const double dt);

    GET_SET(_dt, integration_period);

    template <typename Rep, typename Period>
    void set_integration_period(const std::chrono::duration<Rep, Period>& dt);

    void set_integration_frequency(const double& freq_hz);
};

template <typename Rep, typename Period>
void
Integrable::set_integration_period(const std::chrono::duration<Rep, Period>& dt) {
    using std::chrono::duration_cast;
    using std::chrono::nanoseconds;
    set_integration_period(duration_cast<nanoseconds>(dt).count() * 1e-9);
}

}  // namespace dmp

#endif  // DMPLIB_INTEGRABLE_CLASS_HPP__
