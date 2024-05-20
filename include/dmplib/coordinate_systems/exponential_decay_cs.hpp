#ifndef DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP__
#define DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP__

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/coordinate_systems/coordinate_system.hpp"

namespace dmp {

class ExponentialDecayCs : public CoordinateSystem<ExponentialDecayCs, POINT_TO_POINT> {
protected:
    double _alpha;

public:
    ExponentialDecayCs(const double& alpha);

    GET_SET(_alpha, alpha);

protected:
    // CRTP traits definition
    friend class Integrable<ExponentialDecayCs>;
    void step_impl();
};

}  // namespace dmp


#endif  // DMPLIB_EXPONENTIAL_DECAY_COORDINATE_SYSTEM_HPP__
