#ifndef DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP
#define DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP

#include <algorithm>
#include <cmath>

#include "dmplib/transformation_systems/transformation_system.hpp"

namespace dmp {

template <typename Manifold>
class SecondOrderTf : public TransformationSystem<SecondOrderTf<Manifold>, Manifold> {
public:
    using TF = TransformationSystem<SecondOrderTf<Manifold>, Manifold>;  // NOLINT
    using typename TF::constdoubleRef;
    using typename TF::Forcing_t;
    using typename TF::M;
    using Tangent_t             = typename M::Tangent_t;
    using PosVelAccSample_t     = typename M::PosVelAccSample;
    using PosVelAccTrajectory_t = typename M::PosVelAccTrajectory;

    SecondOrderTf(const constdoubleRef& t) :
            TF::TransformationSystem(t),
            _alpha(48.0),       // NOLINT
            _beta(48.0 / 4.0),  // NOLINT
            _z(this->_M.construct_tangent()){};

    [[nodiscard]] inline double
    get_alpha() const {
        return _alpha;
    }

    inline void
    set_alpha(const double& alpha) {
        _alpha = std::max(0.0, alpha);
    }

    [[nodiscard]] inline double
    get_beta() const {
        return _beta;
    }

    inline void
    set_beta(const double& beta) {
        _beta = std::max(0.0, beta);
    }

    [[nodiscard]] inline Tangent_t
    get_scaled_velocity() const {
        return _z;
    };

    inline void
    set_scaled_velocity(const Tangent_t& z) {
        _z = z;
    };

protected:
    using Integral = Integrable<SecondOrderTf<Manifold>>;  // NOLINT

    // System parameters
    double _alpha; // NOLINT
    double _beta;

    // System states
    Tangent_t _z;

    // Trait implementation
    friend class Integrable<SecondOrderTf<Manifold>>;
    friend class TransformationSystem<SecondOrderTf<Manifold>, Manifold>;

    void
    step_impl() {
        const auto      pos_term = this->_M.logarithmic_map(this->_g, this->_y);
        const Tangent_t dz_dt    = _alpha * (2 * _beta * pos_term - _z) + this->_f;
        const Tangent_t dy_dt    = _z;
        this->_z += dz_dt * this->_dt / this->_T;
        this->_y = this->_M.exponential_map(this->_y, _z);
    }

    Forcing_t
    forcing_term_from_demonstration_impl(const PosVelAccSample_t& demonstration) {
        const auto acc_term = std::pow(this->_T, 2.0) * std::get<2>(demonstration);
        const auto pos_term =
                this->_M.logarithmic_map(this->_g, std::get<0>(demonstration));
        const auto vel_term = this->_T * std::get<1>(demonstration);
        return acc_term - _alpha * (2 * _beta * pos_term + vel_term);
    };
};


}  // namespace dmp

#endif  // DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP
