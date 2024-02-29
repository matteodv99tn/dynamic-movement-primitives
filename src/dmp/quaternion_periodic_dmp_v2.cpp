#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <iostream>

#include "dmp/quaternion_periodic_dmp2.hpp"

using dmp::QuaternionPeriodicDmp2;

//  ____  _        _   _
// / ___|| |_ __ _| |_(_) ___ ___
// \___ \| __/ _` | __| |/ __/ __|
//  ___) | || (_| | |_| | (__\__ \
// |____/ \__\__,_|\__|_|\___|___/
//

static Eigen::Vector3d quaternion_log(const Eigen::Quaterniond& q) {
    const double          nu = q.w();
    const Eigen::Vector3d u  = q.vec();

    const double acos_nu = std::acos(nu);
    if (u.norm() < 1e-6) return Eigen::Vector3d::Zero();

    return acos_nu * u / u.norm();
}

static Eigen::Vector3d quaternion_log(
        const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2
) {
    const Eigen::Quaterniond q = q1 * q2.conjugate();
    return quaternion_log(q);
}

static Eigen::Quaterniond quaternion_exp(const Eigen::Vector3d& v) {
    const double v_norm = v.norm();
    if (v_norm < 1e-6) return Eigen::Quaterniond::Identity();

    const double          sin_v_norm = std::sin(v_norm);
    const double          cos_v_norm = std::cos(v_norm);
    const Eigen::Vector3d u          = sin_v_norm * v / v_norm;

    return Eigen::Quaterniond(cos_v_norm, u.x(), u.y(), u.z());
}

static Eigen::Quaterniond quaternion_exp(
        const Eigen::Quaterniond& q, const Eigen::Vector3d& v
) {
    return quaternion_exp(v) * q;
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

QuaternionPeriodicDmp2::QuaternionPeriodicDmp2(
        const BasisFunction::SharedPtr& basis,
        const double                    alpha,
        const double                    lambda,
        const double                    dt
) :
        DmpBase(basis, alpha, lambda, dt) {
    _q = Eigen::Quaterniond::Identity();
    _g = Eigen::Quaterniond::Identity();
    // _y     = Eigen::Vector3d::Zero();
    _omega = Eigen::Vector3d::Zero();
    _dz_dt = Eigen::Vector3d::Zero();
    _phi   = 0.0;
    resetWeights();
}

void QuaternionPeriodicDmp2::setObservationPeriod(const double T) {
    _tau = T / (2.0 * M_PI);
}

void QuaternionPeriodicDmp2::setSamplingPeriod(const double dt) { _dt = dt; }

void QuaternionPeriodicDmp2::setTau(const double tau) {
    _tau = tau;
    return;
}

void QuaternionPeriodicDmp2::resetWeights() {
    _w = Eigen::MatrixXd::Zero(_N(), 3);
    _P.resize(6);
    for (std::size_t i = 0; i < 3; ++i) _P[i] = Eigen::MatrixXd::Identity(_N(), _N());
}

void QuaternionPeriodicDmp2::incrementalLearn(
        const double&             phi,
        const Eigen::Quaterniond& q,
        const Eigen::Vector3d&    omega,
        const Eigen::Vector3d&    alpha
) {
    const Eigen::VectorXd basis = (*_basis)(phi);
    const Eigen::VectorXd fd =
            alpha * _tau - _alpha * (2 * _beta * quaternion_log(_g, q) - omega);

    for (std::size_t i = 0; i < 3; ++i) {
        const Eigen::VectorXd psi = basis;
        const Eigen::MatrixXd num = _P[i] * psi * psi.transpose() * _P[i];
        const double          den = _lambda + psi.transpose() * _P[i] * psi;
        _P[i]                     = 1 / _lambda * (_P[i] - num / den);
        _w.col(i) += (fd(i) - psi.transpose() * _w.col(i)) * _P[i] * psi;
    }
}

double QuaternionPeriodicDmp2::timeToPhase(const double& t) const {
    return t * _Omega();
}

Eigen::VectorXd QuaternionPeriodicDmp2::timeToPhase(const Eigen::VectorXd& t) const {
    return t * _Omega();
}

void QuaternionPeriodicDmp2::setInitialConditions(
        const Eigen::Quaterniond& q0, const Eigen::Vector3d& omega0, const double& phi0
) {
    _q     = q0;
    _omega = omega0;
    _dz_dt = Eigen::Vector3d::Zero();
    _phi   = phi0;
}

void QuaternionPeriodicDmp2::step() {
    _dz_dt = _Omega() * (_alpha * (2 * _beta * quaternion_log(_g, _q) - _omega) +
                         (*_basis)(_phi, _w));

    _q = quaternion_exp(0.5 * _dt * _Omega() * _omega);
    _q.normalize();
    // std::cout << _q << std::endl;
    _omega += _dz_dt * _dt;
    _phi += _Omega() * _dt;
}

double QuaternionPeriodicDmp2::getPhase() const { return _phi; }

Eigen::Quaterniond QuaternionPeriodicDmp2::getQuaternionState() const { return _q; }

Eigen::Vector3d QuaternionPeriodicDmp2::getAngularVelocityState() const {
    return _omega;
}

Eigen::Vector3d QuaternionPeriodicDmp2::getAngularAcceleration() const {
    return _dz_dt;
}
