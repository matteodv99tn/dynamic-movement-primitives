#ifndef DMPLIB_BASIS_FUNCTION_HPP
#define DMPLIB_BASIS_FUNCTION_HPP

#include <cstddef>
#include <Eigen/Dense>
#include <utility>

#include "range/v3/algorithm/copy.hpp"
#include "range/v3/view/take.hpp"
#include "range/v3/view/transform.hpp"

namespace dmp {

template <typename Derived>
class BasisFunction {
public:
    using Basis_t         = Eigen::VectorXd;
    using Weights_t       = Eigen::VectorXd;
    using WeightsVector_t = std::vector<Eigen::VectorXd>;

    BasisFunction(
            const std::size_t& basis_size,
            const double&      min_support = 0.0,
            const double&      max_support = 1.0,
            const bool&        include_ub  = true
    ) :
            _basis_size(basis_size), _support({min_support, max_support}) {
        static_cast<Derived*>(this)->init_on_support(include_ub);
    }

    // Parent class must implement
    // Basis evalute_impl(const double& arg);
    // Return basis shall not be normalised
    [[nodiscard]] Basis_t
    evaluate(const double& arg, const bool& normalize = true) const {
        Basis_t out = static_cast<const Derived*>(this)->evaluate_impl(arg);
        if (normalize) out /= out.norm();
        return out;
    }

    // Note: the function does not check that the dimension of the basis function is
    // compatible with the given weights, even though this should be enforced by the
    // inherited class.
    [[nodiscard]] double
    evaluate(const double& arg, const Weights_t& weights, const bool& normalize = true)
            const {
        Basis_t base = evaluate(arg, normalize);
        return base.transpose() * weights;
    }

    template <int N = Eigen::Dynamic>
    Eigen::Matrix<double, N, 1>
    evaluate(
            const double&          arg,
            const WeightsVector_t& weights,
            const bool&            normalize = true
    ) const {
        namespace rs = ranges;
        namespace rv = ranges::views;

        Eigen::Matrix<double, N, 1> res;
        std::size_t                 n_elems = N;
        if (N == Eigen::Dynamic) {
            res     = Eigen::VectorXd(weights.size());
            n_elems = weights.size();
        }


        rs::copy(
                weights | rv::transform([this, arg, normalize](const Weights_t& w) {
                    return evaluate(arg, w, normalize);
                }) | rv::take(n_elems),
                res.data()
        );
        return res;
    }

protected:
    std::size_t               _basis_size;
    std::pair<double, double> _support;
};
}  // namespace dmp

#endif  // DMPLIB_BASIS_FUNCTION_HPP
