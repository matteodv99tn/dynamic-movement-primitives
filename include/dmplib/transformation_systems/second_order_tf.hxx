#ifndef DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HXX__
#define DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HXX__

#include <cmath>

#include "dmplib/transformation_systems/second_order_tf.hpp"

namespace dmp {

template <typename M>
SecondOrderTf<M>::SecondOrderTf(const typename TF::constdoubleRef& t) :
        TF::TransformationSystem(t) {
    _alpha = 48.0;
    _beta  = _alpha / 4.0;
    _z     = this->_M.construct_tangent();
}

template <typename M>
void
SecondOrderTf<M>::step_impl() {
    const Tangent_t dz_dt =
            _alpha * (2 * _beta * this->_M.logarithmic_map(this->_g, this->_y) - _z)
            + this->_f;
    const Tangent_t dy_dt = _z;
    _z += dz_dt * this->_dt / this->_T;
    this->_y = this->_M.exponential_map(this->_y, _z);
}

template <typename M>
typename SecondOrderTf<M>::Forcing_t
SecondOrderTf<M>::forcing_term_from_demonstration_impl(
        const typename M::PosVelAccSample& dem
) {
    const auto acc_term = std::pow(this->_T, 2.0) * std::get<2>(dem);
    const auto pos_term = this->_M.logarithmic_map(this->_g, std::get<0>(dem));
    const auto vel_term = this->_T * std::get<1>(dem);
    return acc_term - _alpha * (2 * _beta * pos_term + vel_term);
}

}  // namespace dmp


#endif  // DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HXX__
