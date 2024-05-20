#ifndef DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP__
#define DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP__

#include "dmplib/coordinate_systems/coordinate_system.hpp"

namespace dmp {

class ExponentialDecayCs: public CoordinateSystem {
public:
    static constexpr RepresentationType Type = POINT_TO_POINT;

protected:
    double _alpha;

public:
    ExponentialDecayCs(const double& alpha);

    GET_SET(_alpha, alpha);

    void step();
};

}  // namespace dmp


#endif  // DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP__
