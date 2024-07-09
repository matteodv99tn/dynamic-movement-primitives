#ifndef DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP
#define DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP

#include <Eigen/Dense>

#include "dmplib/manifolds/concepts.hpp"
#include "dmplib/transformation_systems/transformation_system.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"

namespace dmp::transformationsystem {

template <dmp::riemannmanifold::concepts::riemann_manifold M>
class SecondOrderTs : public TransformationSystem<M> {
    using Ts               = TransformationSystem<M>;  // NOLINT: alias case
    using Tangent_t        = Ts::Tangent_t;
    using ConstdoubleRef_t = Ts::ConstdoubleRef_t;


private:
    using Ts::_g;
    using Ts::delta_pos_gain;

    template <int Offset, typename Tpl>
    [[nodiscard]] Tangent_t
    forcing_term_impl(const Tpl& sample, const bool& remove_gain_contribution) const {
        const Tangent_t acc_term =
                std::pow(this->_T, 2.0) * std::get<2 + Offset>(sample);
        const Tangent_t pos_term = logarithmic_map(_g, std::get<0 + Offset>(sample));
        const Tangent_t vel_term = this->_T * std::get<1 + Offset>(sample);
        Tangent_t       forcing = acc_term - _alpha * (2 * _beta * pos_term + vel_term);

        const Tangent_t gain = delta_pos_gain();
        if (remove_gain_contribution) {
            // forcing.array() = forcing.array() / delta_pos_gain().array();
            for (std::size_t i = 0; i < forcing.rows(); ++i) {
                double f   = forcing(i);
                forcing[i] = f / gain(i);
            }
        }

        return forcing;
    }


public:
    SecondOrderTs(ConstdoubleRef_t T) :
            Ts(T), _alpha(48.0), _beta(48.0 / 4){};  // NOLINT

    [[nodiscard]] Tangent_t
    evaluate_forcing_term(
            const dmp::PosVelAccSample_t<M>& sample,
            const bool&                      remove_gain_contribution
    ) const {
        return forcing_term_impl<0>(sample, remove_gain_contribution);
    }

    [[nodiscard]] Tangent_t
    evaluate_forcing_term(
            const dmp::StampedPosVelAccSample_t<M>& sample,
            const bool&                             remove_gain_contribution
    ) const {
        return forcing_term_impl<1>(sample, remove_gain_contribution);
    }

    template <typename Tpl>
    [[nodiscard]] std::vector<Tangent_t>
    evaluate_forcing_term(
            const std::vector<Tpl>& traj, const bool& remove_gain_contribution
    ) const {
        using ::ranges::views::transform;
        return traj
               | transform(
                       [this,
                        remove_gain_contribution](const auto& sample) -> Tangent_t {
                           return evaluate_forcing_term(
                                   sample, remove_gain_contribution
                           );
                       }
               )
               | ::ranges::to_vector;
    }

    template <typename Tpl>
    Eigen::MatrixXd
    evaluate_forcing_term_matrix(
            const std::vector<Tpl>& traj, const bool& remove_gain_contribution
    ) const {
        static_assert(Tangent_t::RowsAtCompileTime != -1);
        Eigen::MatrixXd res(traj.size(), Tangent_t::RowsAtCompileTime);

        for (std::size_t i = 0; i < traj.size(); ++i) {
            res.row(i) = evaluate_forcing_term(  // NOLINT: narrowing conversion on i
                    traj[i],
                    remove_gain_contribution
            );
        }

        return res;
    }

private:
    double _alpha;
    double _beta;
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

#endif  // DMPLIB_SECOND_ORDER_TRANSFORMATION_SYSTEM_HPP
