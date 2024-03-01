#include "dmp/quaternion_periodic_dmp.hpp"

#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>

#include "dmp/quaternion_utils.hpp"

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
    _q = Eigen::Quaterniond::Identity();
    _g = Eigen::Quaterniond::Identity();
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
    const Eigen::MatrixXd q_log = computeLogarithms(q);
    _dmp.batchLearn(phi, -2 * q_log, omega, alpha);
}

void QuaternionPeriodicDmp::step() {
    _dmp.step();

    const Eigen::Vector3d omega = _dmp.getVelocityState();
    Eigen::Quaterniond    q_old = _q;
    _q                          = dmp::exponential_map(0.5 * _dt() * omega, _q);

    if (_q.w() * q_old.w() + _q.vec().dot(q_old.vec()) < 0) {
        std::cout << "Jumping" << std::endl;
        _q.coeffs() = -_q.coeffs();
    }

    _dmp.setPositionState(-2 * dmp::logarithmic_map(_g, _q));
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

Eigen::MatrixXd QuaternionPeriodicDmp::getLearnedForcingFunction(
        const Eigen::VectorXd& phi
) const {
    return _dmp.getLearnedForcingFunction(phi);
}

Eigen::VectorXd QuaternionPeriodicDmp::evaluateDesiredForce(
        const Eigen::Quaterniond& q,
        const Eigen::VectorXd&    omega,
        const Eigen::VectorXd&    alpha
) const {
    Eigen::Vector3d q_log = dmp::logarithmic_map(_g, q);
    return _dmp.evaluateDesiredForce(-2 * q_log, omega, alpha);
}

Eigen::MatrixXd QuaternionPeriodicDmp::evaluateDesiredForce(
        const Eigen::MatrixXd& q,
        const Eigen::MatrixXd& omega,
        const Eigen::MatrixXd& alpha
) const {
    Eigen::MatrixXd q_log = computeLogarithms(q);
    return _dmp.evaluateDesiredForce(-2 * q_log, omega, alpha);
}
