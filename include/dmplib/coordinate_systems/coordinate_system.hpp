#ifndef DMPLIB_COORDINATE_SYSTEM_HPP__
#define DMPLIB_COORDINATE_SYSTEM_HPP__

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/utils/macros.hpp"

namespace dmp {

class CoordinateSystem : public Integrable {
public:
    enum RepresentationType {
        POINT_TO_POINT,
        PERIODIC
    };

    // Each derived class MUST implement something like
    // static constexpr RepresentationType Type = POINT_TO_POINT;

protected:
    double _x;  //< coordinate
    double _T;  //< observation period

public:
    CoordinateSystem(const double& initial_value = 1);

    GET_SET(_x, coordinate);
    GET_SET(_T, observation_period);
};

}  // namespace dmp


#endif  // DMPLIB_COORDINATE_SYSTEM_HPP__
