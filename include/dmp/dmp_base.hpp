#ifndef DMP_DMP_BASE_HPP__
#define DMP_DMP_BASE_HPP__

#include "dmp/basis_function/basis_function.hpp"

namespace dmp {

    class DmpBase {
    public:
        DmpBase(const BasisFunction::SharedPtr& basis,
                const double                    alpha  = 48.0,
                const double                    lambda = 0.999,
                const double                    dt     = 0.002);

    // protected:
    public:
        BasisFunction::SharedPtr _basis;
        double                   _alpha, _beta;
        double                   _lambda;
        double                   _tau;
        double                   _dt;
    };

}  // namespace dmp


#endif  // DMP_DMP_BASE_HPP__
