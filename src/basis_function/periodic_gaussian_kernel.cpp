#include "dmp/basis_function/periodic_gaussian_kernel.hpp"

#include <cmath>

using dmp::PeriodicGaussianKernel;

PeriodicGaussianKernel::PeriodicGaussianKernel(const std::size_t& N) :
        dmp::BasisFunction(N) {
    _c = Eigen::VectorXd::LinSpaced(N, 0, 2 * M_PI - 2 * M_PI / N);
    _h = Eigen::VectorXd::Ones(N) * 2.5 * _N;
    return;
}

dmp::BasisFunction::ArgOutPair PeriodicGaussianKernel::evaluate(const double arg
) const {
    const double    x      = std::fmod(arg, 2 * M_PI);
    Eigen::VectorXd result = Eigen::VectorXd::Zero(_N);
    for (std::size_t i = 0; i < _N; i++) {
        result(i) = std::exp(-_h(i) * std::cos(x - _c(i)));
    }

    double r_sum = result.sum();
    if (r_sum < 1e-6) result = Eigen::VectorXd::Zero(_N);
    else result /= r_sum;
    return std::make_pair(x, result);
}
