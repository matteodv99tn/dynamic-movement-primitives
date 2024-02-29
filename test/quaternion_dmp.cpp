#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/quaternion_periodic_dmp.hpp"
#include "dmp/utils.hpp"

#define BATCH_LEARN

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
    Eigen::Quaterniond q = q1 * q2.conjugate();
    return quaternion_log(q);
}

int main() {
    Eigen::MatrixXd trajectory = dmp::loadTrainingTrajectory();
    Eigen::MatrixXd q_traj     = dmp::getQuaternionTrajectory(trajectory);
    Eigen::MatrixXd omega_traj = dmp::getAngularVelocityTrajectory(trajectory);
    Eigen::MatrixXd alpha_traj = dmp::getAngularAccelerationTrajectory(trajectory);
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(150);

    dmp::QuaternionPeriodicDmp dmp(basis);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    Eigen::Vector4d    q0_elems = q_traj.row(0);
    Eigen::Quaterniond q0(q0_elems);
    dmp.setInitialConditions(q0, omega_traj.row(0));

    std::size_t     t_horizon = time.size() * 1;
    Eigen::MatrixXd Qhist(t_horizon, 4);
    Eigen::MatrixXd Lhist(t_horizon, 3);
    Eigen::MatrixXd Lcomputed(t_horizon, 3);
    // Eigen::MatrixXd Yhist(t_horizon, 3);
    // Eigen::MatrixXd Zhist(t_horizon, 3);
    // Eigen::Vector3d pos, vel, acc;
    Eigen::Quaterniond q;

#ifdef BATCH_LEARN
    dmp.batchLearn(phi, q_traj, omega_traj, alpha_traj);
#endif

    for (int i = 0; i < t_horizon; i++) {
        q            = dmp.getQuaternionState();

        Qhist.row(i) = q.coeffs().transpose();
        Lhist.row(i) = dmp.getLogarithm().transpose();

        if(i > 0) {
            Eigen::Vector4d q1_elems = q_traj.row(i - 1);
            Eigen::Vector4d q2_elems = q_traj.row(i);
            Eigen::Quaterniond q1(q1_elems);
            Eigen::Quaterniond q2(q2_elems);
            Lcomputed.row(i) = quaternion_log(q1, q2).transpose() / 0.002;
        }
#ifndef BATCH_LEARN
        if (i < time.size()) {
            q0_elems = q_traj.row(i);
            q0       = Eigen::Quaterniond(q0_elems);
            dmp.incrementalLearn(phi(i), q0, omega_traj.row(i), alpha_traj.row(i));
        }
#endif
        dmp.step();
        // dmp.step(omega_traj.row(i));
    }

    dmp.batchLearn(phi, q_traj, omega_traj, alpha_traj);

    std::ofstream   file("quaternion.csv");
    Eigen::MatrixXd Data = Eigen::MatrixXd::Zero(t_horizon, 10);
    Data << Qhist, Lhist, dmp.q_log;
    // Data << Qhist, Lhist, Lcomputed;
    file << dmp::dumpToCsv(Data);
    file.close();

    return 0;
}
