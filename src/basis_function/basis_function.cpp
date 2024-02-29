#include "dmp/basis_function/basis_function.hpp"

#include <cstddef>
#include <Eigen/Dense>

using dmp::BasisFunction;

BasisFunction::BasisFunction(const std::size_t& N) : _N(N), _last_arg(-1e9) { return; }

Eigen::VectorXd BasisFunction::operator()(const double& arg) const {
    if (arg == _last_arg) return _last_result;

    auto [_last_arg, _last_result] = evaluate(arg);

    return _last_result;
}

double BasisFunction::operator()(const double& arg, const Eigen::VectorXd& w) const {
    return operator()(arg).dot(w);
}

Eigen::VectorXd BasisFunction::operator()(const double& arg, const Eigen::MatrixXd& w)
        const {
    return (operator()(arg).transpose() * w).transpose();
}

const std::size_t& BasisFunction::N() const { return _N; }
