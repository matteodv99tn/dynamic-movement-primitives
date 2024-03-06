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

double DmpBase::getAlpha() const { return _alpha; }

double DmpBase::getBeta() const { return _beta; }

double DmpBase::getSamplingPeriod() const { return _dt; }

double DmpBase::getTau() const { return _tau; }

void DmpBase::setSamplingPeriod(const double& dt) { _dt = dt; };

void DmpBase::setTau(const double& tau) { _tau = tau; };
