#ifndef DMPLIB_TRANSFORMATION_SYSTEM_HPP__
#define DMPLIB_TRANSFORMATION_SYSTEM_HPP__

#include "dmplib/class_traits/integrable.hpp"

namespace dmp {

template <typename Derived, typename Manifold>
class TransformationSystem : public Integrable<Derived> {
public:
    using M = Manifold;

protected:
    using forcing_type = typename M::Tangent_t;
    forcing_type  _f;
    const double* _T;

    typename M::Domain_t _y;  //< "position" state
    typename M::Domain_t _g;  //< goal


public:
    TransformationSystem() {
        _f = forcing_type::Zero();
        _T = nullptr;

        _y = M::construct_domain();
        _g = M::construct_domain();
    }

    GET_SET(_f, forcing_term);
    SET(_T, observation_period_ptr);

    GET_SET(_y, state);

    GET_SET(_g, goal);
};
}  // namespace dmp


#endif  // DMPLIB_TRANSFORMATION_SYSTEM_HPP__
