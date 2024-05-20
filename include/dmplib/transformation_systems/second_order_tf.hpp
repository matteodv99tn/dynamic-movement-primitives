#ifndef DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP__
#define DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP__

#include <iostream>
#include "dmplib/transformation_systems/transformation_system.hpp"
#include "dmplib/utils/macros.hpp"

namespace dmp {

template <typename Manifold>
class SecondOrderTf : public TransformationSystem<SecondOrderTf<Manifold>, Manifold> {
public:
    using M = Manifold;

protected:
    using TF = TransformationSystem<SecondOrderTf<Manifold>, Manifold>;

    // System parameters
    bool _alpha;
    bool _beta;

    // System states
    typename M::Tangent_t _z;


public:
    SecondOrderTf();

    GET_SET(_alpha, alpha);
    GET_SET(_beta, beta);
    GET_SET(_z, scaled_velocity);
};


}  // namespace dmp


#include "dmplib/transformation_systems/second_order_tf.hxx"

#endif  // DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP__
