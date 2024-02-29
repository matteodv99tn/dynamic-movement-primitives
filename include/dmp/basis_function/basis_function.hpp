#ifndef DMP_BASIS_FUNCTION_HPP__
#define DMP_BASIS_FUNCTION_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <memory>

namespace dmp {

    class BasisFunction {
    public:
        using ArgOutPair = std::pair<double, Eigen::VectorXd>;
        using SharedPtr  = std::shared_ptr<BasisFunction>;

        BasisFunction(const std::size_t& N);

        const std::size_t& N() const;

        Eigen::VectorXd operator()(const double& arg) const;
        double          operator()(const double& arg, const Eigen::VectorXd& w) const;
        Eigen::VectorXd operator()(const double& arg, const Eigen::MatrixXd& w) const;

    protected:
        std::size_t _N;  // number of basis functions

        double          _last_arg;
        Eigen::VectorXd _last_result;

        virtual ArgOutPair evaluate(const double arg) const = 0;
    };
}  // namespace dmp

#endif  // DMP_BASIS_FUNCTION_HPP__
