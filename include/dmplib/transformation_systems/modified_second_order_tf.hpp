#ifndef DMPLIB_MODIFIED_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP
#define DMPLIB_MODIFIED_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP

#include <Eigen/Dense>

#include "dmplib/manifolds/concepts.hpp"
#include "dmplib/manifolds/riemann_manifold.hpp"
#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/time_axis.hpp"
#include "dmplib/transformation_systems/transformation_system.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"
#include "transformation_system.hpp"

namespace dmp::transformationsystem {

template <dmp::riemannmanifold::concepts::riemann_manifold M>
class ModifiedSecondOrderTs : public TransformationSystem<ModifiedSecondOrderTs<M>, M> {
private:
    using Ts        = TransformationSystem<ModifiedSecondOrderTs<M>, M>;  // NOLINT: alias case
    using Domain_t  = M;
    using Tangent_t = dmp::riemannmanifold::tangent_space_t<M>;


    using Ts::_f;
    using Ts::_g;
    using Ts::_y;
    using Ts::_y0;
    using Ts::delta_pos_gain;
    using Ts::dt;
    using Ts::T;

public:
    ModifiedSecondOrderTs(
            ::dmp::TimeAxis*    time_axis,
            const double* const s,
            const double&       alpha        = 48.0,        // NOLINT: magic numbers
            const double&       beta         = 48.0 / 4.0,  // NOLINT: magic numbers
            const bool&         fd_uses_gain = true

    ) :
            _alpha(alpha),
            _beta(beta),
            _consider_delta_gain(fd_uses_gain),
            _s(s),
            dmp::transformationsystem::TransformationSystem<ModifiedSecondOrderTs<M>, M>(
                    time_axis
            ){};  // NOLINT

    [[nodiscard]] Tangent_t
    evaluate_forcing_term(
            const dmp::PosVelAccSample_t<M>& sample, const double* const s = nullptr
    ) const {
        return forcing_term_impl<0>(sample, s);
    }

    [[nodiscard]] Tangent_t
    evaluate_forcing_term(
            const dmp::StampedPosVelAccSample_t<M>& sample,
            const double* const                     s = nullptr
    ) const {
        return forcing_term_impl<1>(sample, s);
    }

    template <typename Tpl>
    [[nodiscard]] std::vector<Tangent_t>
    evaluate_forcing_term(const std::vector<Tpl>& traj, const double* const s = nullptr)
            const {
        using ::ranges::views::transform;
        return traj | transform([this, s](const auto& sample) -> Tangent_t {
                   return evaluate_forcing_term(sample, s);
               })
               | ::ranges::to_vector;
    }

    template <typename Tpl>
    Eigen::MatrixXd
    evaluate_forcing_term_matrix(const std::vector<Tpl>& traj, const Eigen::VectorXd s)
            const {
        static_assert(Tangent_t::RowsAtCompileTime != -1);
        Eigen::MatrixXd res(traj.size(), Tangent_t::RowsAtCompileTime);

        for (std::size_t i = 0; i < traj.size(); ++i) {
            res.row(i) = evaluate_forcing_term(  // NOLINT: narrowing conversion on i
                    traj[i],
                    &s(i)
            );
        }

        return res;
    }

    void
    reset_velocity_state() {
        _z = Tangent_t::Zero();
    }

    void
    enable_force_scaling() {
        _consider_delta_gain = true;
    }

    void
    disable_force_scaling() {
        _consider_delta_gain = false;
    }


private:
    // friend class Integrable<SecondOrderTf<M>>;
    friend class TransformationSystem<ModifiedSecondOrderTs<M>, M>;

    template <int Offset, typename Tpl>
    [[nodiscard]] Tangent_t
    forcing_term_impl(const Tpl& sample, const double* const s) const {
        double s_value = (s != nullptr) ? *s : *_s;
        using ::dmp::riemannmanifold::logarithmic_map;
        const M&         y   = std::get<0 + Offset>(sample);
        const Tangent_t& yd  = std::get<1 + Offset>(sample);
        const Tangent_t& ydd = std::get<2 + Offset>(sample);
        const auto log_coeff = ::dmp::riemannmanifold::constants<M>::log_coefficient();

        const Tangent_t log_gy  = log_coeff * logarithmic_map(y, _g);
        const Tangent_t log_gy0 = log_coeff * logarithmic_map(_y0, _g);

        Tangent_t forcing = (T() * T() * ydd / _alpha + T() * yd) / _beta - log_gy
                            + log_gy0 * s_value;

        if (_consider_delta_gain) {
            const Tangent_t gain = delta_pos_gain();
            // forcing.array() = forcing.array() / delta_pos_gain().array();
            for (long i = 0; i < forcing.rows(); ++i) {
                double f   = forcing(i);
                forcing[i] = f / gain(i);
            }
        }
        return forcing;
    }

    void
    step_impl() {
        using ::dmp::riemannmanifold::exponential_map;
        using ::dmp::riemannmanifold::logarithmic_map;

        const auto log_coeff = ::dmp::riemannmanifold::constants<M>::log_coefficient();
        const auto exp_coeff = ::dmp::riemannmanifold::constants<M>::exp_coefficient();

        const Tangent_t log_gy  = log_coeff * logarithmic_map(_y, _g);
        const Tangent_t log_gy0 = log_coeff * logarithmic_map(_y0, _g);
        // _dz_dt                  = _alpha * (2 * _beta * pos_term - _z) + this->_f;
        Tangent_t forcing = this->_f;
        if (_consider_delta_gain) {
            const Tangent_t gain = this->delta_pos_gain();
            for (long i = 0; i < Tangent_t::RowsAtCompileTime; i++)
                forcing(i) *= gain(i);
        }

        _dz_dt = _alpha * (_beta * (log_gy - log_gy0 * (*_s) + forcing) - _z);
        _z += _dz_dt * dt() / T();
        _y = exponential_map(_y, exp_coeff * _z * dt() / T());
    }

    double _alpha;
    double _beta;

public:
    Tangent_t           _z;
    Tangent_t           _dz_dt;
    bool                _consider_delta_gain;
    const double* const _s;
};


}  // namespace dmp::transformationsystem

/*
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
*/

#endif  // DMPLIB_MODIFIED_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP
