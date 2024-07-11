#ifndef DMPLIB_WEIGHTED_BASIS_FUNCTION_HPP
#define DMPLIB_WEIGHTED_BASIS_FUNCTION_HPP

#include <Eigen/Dense>
#include <Eigen/src/QR/ColPivHouseholderQR.h>
#include <iostream>

#include "dmplib/manifolds/concepts.hpp"
#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::learnablefunction {

template <typename Basis, dmp::riemannmanifold::concepts::riemann_manifold M>
class WeightedBasisFunction {
    // The weighted basis uses the same "Basis" on different weights to handle different
    // manifold types

    using Tangent_t = dmp::riemannmanifold::tangent_space_t<M>;

public:
    WeightedBasisFunction(const Basis& basis) : _basis(basis) {
        for (auto& w : _ws) w = Eigen::VectorXd::Zero(basis.size());
    };

    ~WeightedBasisFunction() = default;

    void
    enable_basis_normalisation() {
        _use_normalisation = true;
    }

    void
    disable_basis_normalisation() {
        _use_normalisation = false;
    }

    [[nodiscard]] Tangent_t
    evaluate(const double& arg) {
        Tangent_t res;
        auto      b = _basis.evaluate(arg, true);
        for (std::size_t i = 0; i < w_count; i++) { res(i) = b.transpose() * _ws[i]; }
        return res;
    }

    void
    learn(const Eigen::VectorXd& args, const Eigen::MatrixXd& desired_function) {
        const std::size_t n_dems = args.rows();
        assert(static_cast<std::size_t>(desired_function.rows()) == n_dems);
        assert(desired_function.cols() == w_count);

        Eigen::MatrixXd phi(n_dems, _basis.size());
        for (long i = 0; i < static_cast<long>(n_dems); i++) {
            phi.row(i) = _basis.evaluate(args[i], _use_normalisation);
        }
        const Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr_factorisation(phi);

        for (long i = 0; i < w_count; i++) {
            _ws[i] = qr_factorisation.solve(desired_function.col(i));
        }
    }

    void
    set_function_centers(const std::vector<double>& c) {
        _basis.set_function_centers(c);
    }

private:
    static constexpr int w_count =
            dmp::riemannmanifold::tangent_space_dimension<M>::value;

    Basis                                _basis;
    std::array<Eigen::VectorXd, w_count> _ws;
    bool                                 _use_normalisation{true};
};

}  // namespace dmp::learnablefunction


#endif  // DMPLIB_WEIGHTED_BASIS_FUNCTION_HPP
