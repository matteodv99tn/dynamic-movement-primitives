#include "dmp/periodic_dmp.hpp"

#include <cmath>
#include <cstddef>

using dmp::PeriodicDmp;

PeriodicDmp::PeriodicDmp(
        const BasisFunction::SharedPtr& basis,
        const double                    alpha,
        const double                    lambda,
        const double                    dt
) :
        DmpBase(basis, alpha, lambda, dt) {
    _r = 1.0;
    _w = Eigen::VectorXd::Zero(_N());
    _P = Eigen::MatrixXd::Identity(_N(), _N());
}

void PeriodicDmp::setObservationPeriod(const double T) { _tau = T / (2.0 * M_PI); }

void PeriodicDmp::setSamplingPeriod(const double dt) { _dt = dt; }

void PeriodicDmp::setTau(const double tau) {
    _tau = tau;
    return;
}

void PeriodicDmp::resetWeights() {
    _w = Eigen::VectorXd::Zero(_N());
    _P = Eigen::MatrixXd::Identity(_N(), _N());
}

void PeriodicDmp::incrementalLearn(
        const double& phi, const double& y, const double& dy, const double& ddy
) {
    const Eigen::VectorXd psi = _r * (*_basis)(phi);
    const double fd = ddy * std::pow(_tau, 2) - _alpha * (-_beta * y - dy * _tau);

    const Eigen::MatrixXd num = _P * psi * psi.transpose() * _P;
    const double          den = _lambda + psi.transpose() * _P * psi;
    _P                        = 1 / _lambda * (_P - num / den);
    _w += (fd - psi.transpose() * _w) * _P * psi;
}

void PeriodicDmp::incrementalLearn(const double& phi, const Eigen::Vector3d& y_data) {
    incrementalLearn(phi, y_data(0), y_data(1), y_data(2));
    return;
}

void PeriodicDmp::batchLearn(
        const Eigen::VectorXd& phi,
        const Eigen::VectorXd& y,
        const Eigen::VectorXd& dy,
        const Eigen::VectorXd& ddy
) {
    const std::size_t n_samples = phi.size();
    Eigen::MatrixXd   Phi       = Eigen::MatrixXd::Zero(n_samples, _N());
    for (std::size_t i = 0; i < n_samples; ++i) Phi.row(i) = _r * (*_basis)(phi(i));

    Eigen::VectorXd fd = ddy * std::pow(_tau, 2) - _alpha * (-_beta * y - dy * _tau);
    _w                 = Phi.colPivHouseholderQr().solve(fd);
}

void PeriodicDmp::batchLearn(
        const Eigen::VectorXd& phi, const Eigen::MatrixXd& y_data
) {
    batchLearn(phi, y_data.col(0), y_data.col(1), y_data.col(2));
}

double PeriodicDmp::timeToPhase(const double& t) const { return t * _Omega(); }

Eigen::VectorXd PeriodicDmp::timeToPhase(const Eigen::VectorXd& t) const {
    return t * _Omega();
}

void PeriodicDmp::setInitialConditions(
        const double y0, const double dy0, const double phi0
) {
    _y   = y0;
    _z   = dy0;
    _phi = phi0;
}

void PeriodicDmp::setInitialConditions(const Eigen::Vector2d& y0, const double phi0) {
    setInitialConditions(y0(0), y0(1), phi0);
}

void PeriodicDmp::step() {
    _dz_dt = _Omega() * (_alpha * (-_beta * _y - _z) + _r * (*_basis)(_phi, _w));
    _y += _z * _dt;
    _z += _dz_dt * _dt;
    _phi += _Omega() * _dt;
}

Eigen::Vector3d PeriodicDmp::getState() const {
    return Eigen::Vector3d(_y, _z * _Omega(), _dz_dt * _Omega());
}

double PeriodicDmp::getPhase() const { return _phi; }
