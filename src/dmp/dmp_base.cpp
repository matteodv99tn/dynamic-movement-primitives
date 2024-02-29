#include "dmp/dmp_base.hpp"

using dmp::DmpBase;

DmpBase::DmpBase(
        const BasisFunction::SharedPtr& basis,
        const double                    alpha,
        const double                    lambda,
        const double                    dt
) {
    _basis  = basis;
    _alpha  = alpha;
    _beta   = _alpha / 4.0;
    _lambda = lambda;
    _tau    = 1.0;
    _dt     = dt;
}
