#ifndef DMPLIB_TIME_AXIS_HPP
#define DMPLIB_TIME_AXIS_HPP

#include <chrono>
#include <functional>

#include "dmplib/manifolds/aliases.hpp"

namespace dmp {

class TimeAxis {
public:
    using Reference = std::reference_wrapper<TimeAxis>;  // NOLINT

    [[nodiscard]] double
    get_time() const {
        return _time;
    }

    [[nodiscard]] double
    get_period() const {
        return _period;
    }

    [[nodiscard]] double
    get_timestep() const {
        return _timestep;
    }

    void
    reset_time() {
        _time = 0;
    }

    void
    set_time(const double& t) {
        _time = t;
    }

    void
    set_period(const double& T) {
        _period = T;
    }

    void
    set_posix_period(const std::size_t& T) {
        set_period(static_cast<double>(T * 1e-9));
    }

    void
    set_integration_timestep(const double& dt) {
        _timestep = dt;
    }

    template <typename Rep, typename Period>
    void
    set_integration_timestep(const std::chrono::duration<Rep, Period>& dt) {
        using std::chrono::duration_cast;
        using std::chrono::nanoseconds;
        const double ns_to_s = 1e-9;
        set_integration_timestep(duration_cast<nanoseconds>(dt).count() * ns_to_s);
    }

    void
    set_integration_frequency(const double& freq_hz) {
        _timestep = 1 / freq_hz;
    }

    [[nodiscard]] TimeStamp_t
    time_as_timestamp() const {
        const double s_to_ns = 1e9;
        return static_cast<TimeStamp_t>(_time * s_to_ns);
    }

    void
    step() {
        _time += _timestep;
    }


private:
    double _time{0};
    double _period{0};
    double _timestep{0};
};

}  // namespace dmp


#endif  // DMPLIB_TIME_AXIS_HPP
