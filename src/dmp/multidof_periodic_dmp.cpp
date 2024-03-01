#include "dmp/multidof_periodic_dmp.hpp"

#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>

using dmp::MultiDofPeriodicDmp;

MultiDofPeriodicDmp::MultiDofPeriodicDmp(
        const BasisFunction::SharedPtr& basis,
        const std::size_t               n_dof,
        const double                    alpha,
        const double                    lambda,
        const double                    dt
) :
        DmpBase(basis, alpha, lambda, dt) {
    _n_dof = n_dof;
    _r     = Eigen::VectorXd::Ones(n_dof);
    _y     = Eigen::VectorXd::Zero(n_dof);
    _z     = Eigen::VectorXd::Zero(n_dof);
    _dz_dt = Eigen::VectorXd::Zero(_n_dof);
    _phi   = 0.0;
    resetWeights();
}

void MultiDofPeriodicDmp::setObservationPeriod(const double T) {
    _tau = T / (2.0 * M_PI);
}

void MultiDofPeriodicDmp::resetWeights() {
    _w = Eigen::MatrixXd::Zero(_N(), _n_dof);
    _P.resize(_n_dof);
    for (std::size_t i = 0; i < _n_dof; ++i)
        _P[i] = Eigen::MatrixXd::Identity(_N(), _N());
}

void MultiDofPeriodicDmp::incrementalLearn(
        const double&          phi,
        const Eigen::VectorXd& y,
        const Eigen::VectorXd& dy,
        const Eigen::VectorXd& ddy
) {
    const Eigen::VectorXd basis = (*_basis)(phi);
    // const Eigen::VectorXd fd =
    //         ddy * std::pow(_tau, 2) - _alpha * (-_beta * y - dy * _tau);
    const Eigen::VectorXd fd = evaluateDesiredForce(y, dy, ddy);

    for (std::size_t i = 0; i < _n_dof; ++i) {
        const Eigen::VectorXd psi = basis;
        const Eigen::MatrixXd num = _P[i] * psi * psi.transpose() * _P[i];
        const double          den = _lambda + psi.transpose() * _P[i] * psi;
        _P[i]                     = 1 / _lambda * (_P[i] - num / den);
        _w.col(i) += (fd(i) - psi.transpose() * _w.col(i)) * _P[i] * psi;
    }
}

void MultiDofPeriodicDmp::batchLearn(
        const Eigen::VectorXd& phi,
        const Eigen::MatrixXd& y,
        const Eigen::MatrixXd& dy,
        const Eigen::MatrixXd& ddy
) {
    const std::size_t n_samples = phi.size();
    Eigen::MatrixXd   Phi       = Eigen::MatrixXd::Zero(n_samples, _N());
    for (std::size_t i = 0; i < n_samples; ++i) Phi.row(i) = (*_basis)(phi(i));

    // Eigen::MatrixXd fd = ddy * std::pow(_tau, 2) - _alpha * (-_beta * y - dy * _tau);
    const Eigen::MatrixXd fd = evaluateDesiredForce(y, dy, ddy);
    _w                       = Phi.colPivHouseholderQr().solve(fd);
}

double MultiDofPeriodicDmp::timeToPhase(const double& t) const { return t * _Omega(); }

Eigen::VectorXd MultiDofPeriodicDmp::timeToPhase(const Eigen::VectorXd& t) const {
    return t * _Omega();
}

void MultiDofPeriodicDmp::setInitialConditions(
        const Eigen::VectorXd& y0, const Eigen::VectorXd& dy0, const double& phi0
) {
    _y   = y0;
    _z   = dy0;
    _phi = phi0;
}

void MultiDofPeriodicDmp::step() {
    static std::size_t i = 0;
    _dz_dt = _Omega() * (_alpha * (-_beta * _y - _z) + (*_basis)(_phi, _w));
    _z += _dz_dt * _dt;
    _y += _Omega() * _z * _dt;
    _phi += _Omega() * _dt;
}

double MultiDofPeriodicDmp::getPhase() const { return _phi; }

void MultiDofPeriodicDmp::setPositionState(const Eigen::VectorXd& y) { _y = y; }

Eigen::VectorXd MultiDofPeriodicDmp::getPositionState() const { return _y; }

Eigen::VectorXd MultiDofPeriodicDmp::getVelocityState() const { return _z * _Omega(); }

Eigen::VectorXd MultiDofPeriodicDmp::getZ() const { return _z; }

double MultiDofPeriodicDmp::getOmega() const { return _Omega(); }

Eigen::VectorXd MultiDofPeriodicDmp::getAccelerationState() const {
    return _dz_dt * _Omega();
}

Eigen::VectorXd MultiDofPeriodicDmp::evaluateDesiredForce(
        const Eigen::VectorXd& y, const Eigen::VectorXd& dy, const Eigen::VectorXd& ddy
) const {
    return ddy * std::pow(_tau, 2) - _alpha * (-_beta * y - dy * _tau);
}

Eigen::MatrixXd MultiDofPeriodicDmp::evaluateDesiredForce(
        const Eigen::MatrixXd& y, const Eigen::MatrixXd& dy, const Eigen::MatrixXd& ddy
) const {
    return ddy * std::pow(_tau, 2) - _alpha * (-_beta * y - dy * _tau);
}

Eigen::MatrixXd MultiDofPeriodicDmp::getLearnedForcingFunction(
        const Eigen::VectorXd& phi
) const {
    return (*_basis)(phi, _w);
}
