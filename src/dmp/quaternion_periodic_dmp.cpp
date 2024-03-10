#include "dmp/quaternion_periodic_dmp.hpp"

#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include "dmp/quaternion_utils.hpp"

using dmp::QuaternionPeriodicDmp;

QuaternionPeriodicDmp::QuaternionPeriodicDmp(
        const BasisFunction::SharedPtr& basis,
        const double                    alpha,
        const double                    lambda,
        const double                    dt
) :
        DmpBase::DmpBase(basis, alpha, lambda, dt) {
    _q = Eigen::Quaterniond::Identity();
    _g = Eigen::Quaterniond::Identity();

    _eta     = Eigen::VectorXd::Zero(3);
    _deta_dt = Eigen::VectorXd::Zero(3);
    _phi     = 0.0;
    resetWeights();
}

void QuaternionPeriodicDmp::setObservationPeriod(const double T) { _tau = T; }

void QuaternionPeriodicDmp::resetWeights() {
    _w = Eigen::MatrixXd::Zero(_N(), 3);
    _P.resize(3);
    for (std::size_t i = 0; i < 3; ++i) _P[i] = Eigen::MatrixXd::Identity(_N(), _N());
}

double QuaternionPeriodicDmp::timeToPhase(const double& t) const {
    return t * _Omega() * 2 * M_PI;
}

Eigen::VectorXd QuaternionPeriodicDmp::timeToPhase(const Eigen::VectorXd& t) const {
    return t * _Omega() * 2 * M_PI;
}

double QuaternionPeriodicDmp::getPhase() const { return _phi; }

Eigen::Quaterniond QuaternionPeriodicDmp::getQuaternionState() const { return _q; }

Eigen::Vector3d QuaternionPeriodicDmp::getAngularVelocityState() const {
    return _eta / _tau;
}

Eigen::Vector3d QuaternionPeriodicDmp::getAngularAcceleration() const {
    return _deta_dt / _tau;
}

Eigen::MatrixXd QuaternionPeriodicDmp::getLearnedForcingFunction(
        const Eigen::VectorXd& phi
) const {
    return (*_basis)(phi, _w);
}

Eigen::Vector3d QuaternionPeriodicDmp::getLogarithm() const {
    return dmp::logarithmic_map(_g, _q);
}

Eigen::MatrixXd QuaternionPeriodicDmp::computeLogarithms(const Eigen::MatrixXd& Qs
) const {
    Eigen::MatrixXd q_log = Eigen::MatrixXd::Zero(Qs.rows(), 3);
    Eigen::Vector3d _tmp;
    for (int i = 0; i < Qs.rows(); ++i) {
        Eigen::Vector4d    q_tmp_value = Qs.row(i);
        Eigen::Quaterniond q_tmp(q_tmp_value);
        _tmp         = dmp::logarithmic_map(_g, q_tmp);
        q_log.row(i) = _tmp;
    }
    return q_log;
}

Eigen::VectorXd QuaternionPeriodicDmp::evaluateDesiredForce(
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d&    omega,
        const Eigen::Vector3d&    domega_dt
) const {
    return domega_dt * std::pow(_tau, 2) -
           _alpha * (2 * _beta * dmp::logarithmic_map(_g, q) - _tau * omega);
}

Eigen::MatrixXd QuaternionPeriodicDmp::evaluateDesiredForce(
        const Eigen::MatrixXd& q,
        const Eigen::MatrixXd& omega,
        const Eigen::MatrixXd& domega_dt
) const {
    Eigen::MatrixXd fd(q.rows(), 3);
    for (std::size_t i = 0; i < q.rows(); ++i) {
        Eigen::Quaterniond q_i(Eigen::Vector4d(q.row(i)));
        fd.row(i) = evaluateDesiredForce(
                q_i, Eigen::Vector3d(omega.row(i)), Eigen::Vector3d(domega_dt.row(i))
        );
    }
    return fd;
}

void QuaternionPeriodicDmp::setInitialConditions(
        const Eigen::Quaterniond& q0, const Eigen::Vector3d& omega0, const double& phi0
) {
    _q   = q0;
    _q0  = q0;
    _eta = omega0 * _tau;
    _phi = phi0;
}

void QuaternionPeriodicDmp::incrementalLearn(
        const double&             phi,
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d&    omega,
        const Eigen::Vector3d&    domega_dt
) {
    const Eigen::VectorXd basis = (*_basis)(phi);
    const Eigen::VectorXd fd    = evaluateDesiredForce(q, omega, domega_dt);

    for (std::size_t i = 0; i < 3; ++i) {
        const Eigen::VectorXd psi = basis;
        const Eigen::MatrixXd num = _P[i] * psi * psi.transpose() * _P[i];
        const double          den = _lambda + psi.transpose() * _P[i] * psi;
        _P[i]                     = 1 / _lambda * (_P[i] - num / den);
        _w.col(i) += (fd(i) - psi.transpose() * _w.col(i)) * _P[i] * psi;
    }
    _w_train = _w;
}

void QuaternionPeriodicDmp::incrementalLearn(
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d&    omega,
        const Eigen::Vector3d&    domega_dt
) {
    return incrementalLearn(_phi, q, omega, domega_dt);
}

void QuaternionPeriodicDmp::batchLearn(
        const Eigen::VectorXd& phi,
        const Eigen::MatrixXd& y,
        const Eigen::MatrixXd& dy,
        const Eigen::MatrixXd& ddy
) {
    _eta_hist                   = Eigen::MatrixXd::Zero(phi.size(), 3);
    _deta_dt_hist               = Eigen::MatrixXd::Zero(phi.size(), 3);
    _q_hist                     = Eigen::MatrixXd::Zero(phi.size(), 4);
    _f_hist                     = Eigen::MatrixXd::Zero(phi.size(), 3);
    _log_hist                   = Eigen::MatrixXd::Zero(phi.size(), 3);
    _phi_hist                   = Eigen::VectorXd::Zero(phi.size());
    const std::size_t n_samples = phi.size();
    Eigen::MatrixXd   Phi       = Eigen::MatrixXd::Zero(n_samples, _N());
    for (std::size_t i = 0; i < n_samples; ++i) Phi.row(i) = (*_basis)(phi(i));

    Eigen::Quaterniond q1(Eigen::Vector4d(y.row(0)));

    const Eigen::MatrixXd fd        = evaluateDesiredForce(y, dy, ddy);
    auto                  PhiSolver = Phi.colPivHouseholderQr();
    _w                              = PhiSolver.solve(fd);
    _fd_des                         = fd;
}

void QuaternionPeriodicDmp::step() {
    Eigen::Vector3d log = dmp::logarithmic_map(_g, _q);
    Eigen::Vector3d f   = (*_basis)(_phi, _w);
    _deta_dt            = _Omega() * (_alpha * (2 * _beta * log - _eta) + f);
    _eta += _deta_dt * _dt;

    _q = dmp::exponential_map(0.5 * _dt * _Omega() * _eta, _q);

    if (idx < _eta_hist.rows()) {
        _eta_hist.row(idx)       = _eta;
        _deta_dt_hist.row(idx)   = _deta_dt;
        _q_hist.row(idx)         = Eigen::Vector4d(_q.w(), _q.x(), _q.y(), _q.z());
        _f_hist.row(idx)[0]      = f[0];
        _f_hist.row(idx)[1]      = f[1];
        _f_hist.row(idx)[2]      = f[2];
        _log_hist.row(idx)       = log;
        volatile double phi_curr = _phi;
        _phi_hist(idx)           = phi_curr;
    }

    _phi += _Omega() * 2 * M_PI * _dt;
    if (_phi >= 2 * M_PI) {
        _phi = 0;
        _q   = _q0;
        _eta = Eigen::Vector3d::Zero();
    }
    idx++;
}
