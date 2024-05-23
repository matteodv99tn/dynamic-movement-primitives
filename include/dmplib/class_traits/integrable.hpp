#ifndef DMPLIB_INTEGRABLE_CLASS_HPP
#define DMPLIB_INTEGRABLE_CLASS_HPP

#include <chrono>

#include "dmplib/utils/constants.hpp"

namespace dmp {

template <typename Derived>
class Integrable {
public:
    Integrable(const double& dt = dmp::defaults::integration_period);

    // This function requires the derived class to implement
    // void step_impl();
    void
    step() {
        static_cast<Derived*>(this)->step_impl();
    }

    [[nodiscard]] inline double
    get_integration_period() const {
        return _dt;
    }

    inline void
    set_integration_period(const double& dt) {
        _dt = dt;
    }

    template <typename Rep, typename Period>
    void
    set_integration_period(const std::chrono::duration<Rep, Period>& dt) {
        using std::chrono::duration_cast;
        using std::chrono::nanoseconds;
        const double ns_to_s = 1e-9;
        set_integration_period(duration_cast<nanoseconds>(dt).count() * ns_to_s);
    }

    void set_integration_frequency(const double& freq_hz){
        _dt = 1/freq_hz;
    }

protected:
    double _dt;
};

}  // namespace dmp

#endif  // DMPLIB_INTEGRABLE_CLASS_HPP
