#ifndef DMPLIB_INTEGRABLE_CLASS_HPP__
#define DMPLIB_INTEGRABLE_CLASS_HPP__

#include <chrono>

namespace dmp {

template <typename Derived>
class Integrable {
protected:
    double _dt;

public:
    Integrable(const double& dt = 0.001);

    // This function requires the derived class to implement
    // void step_impl();
    void step();

    inline double
    get_integration_period() const {
        return _dt;
    }

    inline void set_integration_period(const double& dt);

    template <typename Rep, typename Period>
    void set_integration_period(const std::chrono::duration<Rep, Period>& dt);

    void set_integration_frequency(const double& freq_hz);
};

}  // namespace dmp

#include "dmplib/class_traits/integrable.hxx"

#endif  // DMPLIB_INTEGRABLE_CLASS_HPP__
