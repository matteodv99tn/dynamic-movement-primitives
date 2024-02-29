#ifndef DMP_PERIODIC_GAUSSIAN_KERNEL_HPP__
#define DMP_PERIODIC_GAUSSIAN_KERNEL_HPP__

#include <cstddef>
#include <Eigen/Dense>

#include "dmp/basis_function/basis_function.hpp"

namespace dmp {

    class PeriodicGaussianKernel : public BasisFunction {
    public:
        PeriodicGaussianKernel(const std::size_t& N);


    protected:
        Eigen::VectorXd _c;
        Eigen::VectorXd _h;

        virtual ArgOutPair evaluate(const double arg) const override;
    };
}  // namespace dmp

#endif  // DMP_PERIODIC_GAUSSIAN_KERNEL_HPP__
