#ifndef DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HXX__
#define DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HXX__

#include "dmplib/transformation_systems/second_order_tf.hpp"

namespace dmp {

template <typename M>
SecondOrderTf<M>::SecondOrderTf() {
    _alpha = 48.0;
    _beta  = _alpha / 4.0;
    _z     = M::construct_tangent();
}


}  // namespace dmp


#endif  // DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HXX__
