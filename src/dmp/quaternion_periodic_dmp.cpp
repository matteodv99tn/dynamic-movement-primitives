#include "dmp/quaternion_periodic_dmp.hpp"
#include "dmp/quaternion_utils.hpp"

#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

using dmp::QuaternionPeriodicDmp;

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __ ___
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ __|
// | |  | |  __/ | | | | | |_) |  __/ |  \__ \
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|  |___/
//

QuaternionPeriodicDmp::QuaternionPeriodicDmp(
        const BasisFunction::SharedPtr& basis,
        const double                    alpha,
        const double                    lambda,
        const double                    dt
) :
        _dmp(basis, 3, alpha, lambda, dt) {
    _q  = Eigen::Quaterniond::Identity();
    _g  = Eigen::Quaterniond::Identity();
    _dt = dt;

    _alpha = alpha;
    _beta  = _alpha / 4;
}

void QuaternionPeriodicDmp::setObservationPeriod(const double T) {
    _dmp.setObservationPeriod(T);
}

void QuaternionPeriodicDmp::setSamplingPeriod(const double dt) {
    _dmp.setSamplingPeriod(dt);
}

void QuaternionPeriodicDmp::setTau(const double tau) { _dmp.setTau(tau); }

void QuaternionPeriodicDmp::resetWeights() { _dmp.resetWeights(); }

double QuaternionPeriodicDmp::timeToPhase(const double& t) const {
    return _dmp.timeToPhase(t);
}

Eigen::VectorXd QuaternionPeriodicDmp::timeToPhase(const Eigen::VectorXd& t) const {
    return _dmp.timeToPhase(t);
}

void QuaternionPeriodicDmp::incrementalLearn(
        const double&             phi,
        const Eigen::Quaterniond& q,
        const Eigen::VectorXd&    omega,
        const Eigen::VectorXd&    alpha
) {
    const Eigen::Vector3d q_log = dmp::logarithmic_map(_g, q);
    _dmp.incrementalLearn(phi, -2 * q_log, omega, alpha);
}

void QuaternionPeriodicDmp::batchLearn(
        const Eigen::VectorXd& phi,
        const Eigen::MatrixXd& q,
        const Eigen::MatrixXd& omega,
        const Eigen::MatrixXd& alpha
) {
    q_log = Eigen::MatrixXd::Zero(q.rows(), 3);
    Eigen::Vector3d _tmp;
    for (int i = 0; i < q.rows(); ++i) {
        Eigen::Vector4d    q_tmp_value = q.row(i);
        Eigen::Quaterniond q_tmp(q_tmp_value);
        _tmp         = dmp::logarithmic_map(_g, q_tmp);
        q_log.row(i) = -2 * _tmp;
    }

    _fd_store = alpha * std::pow(_dmp._tau, 2) -
                _dmp._alpha * (-_dmp._beta * q_log - omega * _dmp._tau);


    _dmp.batchLearn(phi, q_log, omega, alpha);
}

void QuaternionPeriodicDmp::step() {
    _dmp.step();

    const Eigen::Vector3d omega = _dmp.getVelocityState();
    Eigen::Quaterniond    q_old = _q;
    _q                          = dmp::exponential_map(0.5 * _dt * omega, _q);

    if (_q.w() * q_old.w() + _q.vec().dot(q_old.vec()) < 0) {
        std::cout << "Jumping" << std::endl;
        _q.coeffs() = -_q.coeffs();
    }
}

void QuaternionPeriodicDmp::setInitialConditions(
        const Eigen::Quaterniond& q0, const Eigen::VectorXd& omega0, const double& phi0
) {
    _q = q0;
    _dmp.setInitialConditions(-2 * dmp::logarithmic_map(_g, _q), omega0, phi0);
}

Eigen::Quaterniond QuaternionPeriodicDmp::getQuaternionState() const { return _q; }

Eigen::VectorXd QuaternionPeriodicDmp::getAngularVelocityState() const {
    return _dmp.getVelocityState();
}

Eigen::VectorXd QuaternionPeriodicDmp::getAngularAcceleration() const {
    return _dmp.getAccelerationState();
}

Eigen::Vector3d QuaternionPeriodicDmp::getLogarithm() const {
    return _dmp.getPositionState();
}

/*
void QuaternionPeriodicDmp::integration_quat() {
    Eigen::Vector3d log_tmp = quaternion_log(_g, _q);
    double          *log_q = log_tmp.data();

    for (int i = 0; i < 3; ++i) {
        _do[i] = _alpha * (_beta * 2 * log_q[i] - _z[i]) + _f[i];
        // temporal scaling
        _do[i] = _do[i] / _tau[i];
        _z[i]  = _z[i] + _do[i] * _dt;
    }
    // integration of quaternions
    double norm_do = 0;
    vnorm(_do, norm_do);
    if (norm_do > 1.0e-12) {
        double tmp[]    = {_z[0] / _tau[0], _z[1] / _tau[1], _z[2] / _tau[2]};
        double norm_tmp = 0;
        vnorm(tmp, norm_tmp);
        double norm_z = 0;
        vnorm(_z, norm_z);
        double dq[] = {0, 0, 0, 0};
        dq[0]       = cos(norm_tmp * _dt / 2);
        double ss   = sin(norm_tmp * _dt / 2);
        for (int i = 0; i < _dof; ++i) { dq[i + 1] = ss * _z[i] / norm_z; }
        double q[] = {0, 0, 0, 0};
        quat_mult(dq, _y, q);
        for (int i = 0; i < 4; ++i) { _y[i] = q[i]; }
    }
}
*/
