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

Eigen::VectorXd BasisFunction::operator()(
        const Eigen::VectorXd& arg, const Eigen::VectorXd& w
) const {
    Eigen::VectorXd result(arg.size());
    for (std::size_t i = 0; i < arg.size(); ++i) result(i) = operator()(arg(i), w);
    return result;
}

Eigen::MatrixXd BasisFunction::operator()(
        const Eigen::VectorXd& arg, const Eigen::MatrixXd& w
) const {
    Eigen::MatrixXd result(arg.size(), w.cols());
    for (std::size_t i = 0; i < w.cols(); ++i) {
        Eigen::VectorXd col = w.col(i);
        result.col(i)       = operator()(arg, col);
    }
    return result;
}

const std::size_t& BasisFunction::N() const { return _N; }
