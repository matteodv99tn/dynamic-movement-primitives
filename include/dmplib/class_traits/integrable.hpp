#ifndef DMPLIB_INTEGRABLE_CLASS_HPP
#define DMPLIB_INTEGRABLE_CLASS_HPP

#include <chrono>
#include <functional>

#include "dmplib/time_axis.hpp"
#include "dmplib/utils/constants.hpp"

namespace dmp {

template <typename Derived>
class Integrable {
private:
    TimeAxis::Reference _time_axis;

public:
    using StepHandleFun_t = std::function<void(void)>;

    Integrable(TimeAxis::Reference time_axis) : _time_axis(time_axis) {};

    // This function requires the derived class to implement
    // void step_impl();
    void
    step() {
        static_cast<Derived*>(this)->step_impl();
    }

    [[nodiscard]] const TimeAxis&
    time_axis() const {
        return _time_axis.get();
    }

    [[nodiscard]] TimeAxis&
    time_axis() {
        return _time_axis.get();
    }

    StepHandleFun_t
    step_handler() {
        return [this]() { this->step(); };
    }

protected:
    [[nodiscard]] double
    dt() const {
        return time_axis().get_integration_timestep();
    }

    [[nodiscard]] double
    T() const {  // NOLINT
        return time_axis().get_period();
    }

    [[nodiscard]] double
    t() const {
        return time_axis().get_time();
    }
};

}  // namespace dmp

#endif  // DMPLIB_INTEGRABLE_CLASS_HPP
