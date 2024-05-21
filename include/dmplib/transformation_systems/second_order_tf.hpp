#ifndef DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP__
#define DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP__

#include <algorithm>
#include <iostream>

#include "dmplib/transformation_systems/transformation_system.hpp"

namespace dmp {

template <typename Manifold>
class SecondOrderTf : public TransformationSystem<SecondOrderTf<Manifold>, Manifold> {
public:
    using M         = Manifold;
    using Tangent_t = typename M::Tangent_t;

protected:
    using TF       = TransformationSystem<SecondOrderTf<Manifold>, Manifold>;
    using Integral = Integrable<SecondOrderTf<Manifold>>;

    using typename TF::Forcing_t;

    // System parameters
    bool _alpha;
    bool _beta;

    // System states
    Tangent_t _z;


public:
    SecondOrderTf(const typename TF::constdoubleRef& t);

    using PosVelAccSample     = typename M::PosVelAccSample;
    using PosVelAccTrajectory = typename M::PosVelAccTrajectory;


protected:
    friend class Integrable<SecondOrderTf<Manifold>>;
    void step_impl();

    friend class TransformationSystem<SecondOrderTf<Manifold>, Manifold>;
    Forcing_t forcing_term_from_demonstration_impl(
            const PosVelAccSample& demonstration
    );

public:
    inline double
    get_alpha() const {
        return _alpha;
    }

    inline void
    set_alpha(const double& alpha) {
        _alpha = std::max(0.0, alpha);
    }

    inline double
    get_beta() const {
        return _beta;
    }

    inline void
    set_beta(const double& beta) {
        _beta = std::max(0.0, beta);
    }

    inline Tangent_t
    get_scaled_velocity() const {
        return _z;
    };

    inline void
    set_scaled_velocity(const Tangent_t& z) {
        _z = z;
    };
};


}  // namespace dmp

#include "dmplib/transformation_systems/second_order_tf.hxx"

#endif  // DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP__
