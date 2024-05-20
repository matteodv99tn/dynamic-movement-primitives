#ifndef DMPLIB_COORDINATE_SYSTEM_HPP__
#define DMPLIB_COORDINATE_SYSTEM_HPP__

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/utils/macros.hpp"

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

    GET_SET(_x, coordinate);
    GET_SET(_T, observation_period);
};

}  // namespace dmp


#endif  // DMPLIB_COORDINATE_SYSTEM_HPP__
