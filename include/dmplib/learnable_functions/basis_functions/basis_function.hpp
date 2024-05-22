#ifndef DMPLIB_BASIS_FUNCTION_HPP__
#define DMPLIB_BASIS_FUNCTION_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <utility>

namespace dmp {

template <typename Derived>
class BasisFunction {
public:
    using Basis         = Eigen::VectorXd;
    using Weights       = Eigen::VectorXd;
    using WeightsVector = std::vector<Eigen::VectorXd>;

    BasisFunction(
            const std::size_t& basis_size,
            const double&      min_support = 0.0,
            const double&      max_support = 1.0,
            const bool&        include_ub  = true
    );

    // Parent class must implement
    // Basis evalute_impl(const double& arg);
    // Return basis shall not be normalised
    Basis evaluate(const double& arg, const bool& normalize = true) const;

    // Note: the function does not check that the dimension of the basis function is
    // compatible with the given weights, even though this should be enforced by the
    // inherited class.
    double evaluate(
            const double& arg, const Weights& weights, const bool& normalize = true
    ) const;

    template <int N = Eigen::Dynamic>
    Eigen::Matrix<double, N, 1> evaluate(
            const double&        arg,
            const WeightsVector& weights,
            const bool&          normalize = true
    ) const;

protected:
    std::size_t               _basis_size;
    std::pair<double, double> _support;
};
}  // namespace dmp

#include "dmplib/learnable_functions/basis_functions/basis_function.hxx"

#endif  // DMPLIB_BASIS_FUNCTION_HPP__
