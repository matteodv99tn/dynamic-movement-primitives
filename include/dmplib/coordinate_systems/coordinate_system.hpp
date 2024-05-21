#ifndef DMPLIB_COORDINATE_SYSTEM_HPP__
#define DMPLIB_COORDINATE_SYSTEM_HPP__

#include <functional>

#include "dmplib/class_traits/integrable.hpp"

namespace dmp {
enum RepresentationType {
    POINT_TO_POINT,
    PERIODIC
};

template <typename Derived, RepresentationType rep_type = POINT_TO_POINT>
class CoordinateSystem : public Integrable<Derived> {
public:
    static constexpr RepresentationType Type = rep_type;

protected:
    double _x;  //< coordinate
    double _T;  //< observation period

public:
    CoordinateSystem(const double& initial_value = 1) { _x = initial_value; }

    inline double
    get_coordinate() const {
        return _x;
    }

    inline void
    set_coordinate(const double& x) {
        _x = x;
    }

    inline double
    get_observation_period() {
        return _T;
    }

    inline void
    set_observation_period(const double& T) {
        _T = T;
    }

    inline std::reference_wrapper<const double>
    get_observation_period_reference() {
        return std::cref(_T);
    }
};

}  // namespace dmp


#endif  // DMPLIB_COORDINATE_SYSTEM_HPP__
