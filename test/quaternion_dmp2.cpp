#include <cstddef>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/quaternion_periodic_dmp2.hpp"
#include "dmp/utils.hpp"

int main() {
    Eigen::MatrixXd trajectory = dmp::loadTrainingTrajectory();
    Eigen::MatrixXd q_traj     = dmp::getQuaternionTrajectory(trajectory);
    Eigen::MatrixXd omega_traj = dmp::getAngularVelocityTrajectory(trajectory);
    Eigen::MatrixXd alpha_traj = dmp::getAngularAccelerationTrajectory(trajectory);
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(20);

    dmp::QuaternionPeriodicDmp2 dmp(basis);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    Eigen::Quaterniond q0;
    q0.x() = q_traj(0, 0);
    q0.y() = q_traj(0, 1);
    q0.z() = q_traj(0, 2);
    q0.w() = q_traj(0, 3);
    dmp.setInitialConditions(q0, omega_traj.row(0));

    std::size_t     t_horizon = time.size() * 3;
    Eigen::MatrixXd Qhist(t_horizon, 4);
    // Eigen::MatrixXd Yhist(t_horizon, 3);
    // Eigen::MatrixXd Zhist(t_horizon, 3);
    // Eigen::Vector3d pos, vel, acc;
    Eigen::Quaterniond q;

    for (int i = 0; i < t_horizon; i++) {

        q = dmp.getQuaternionState();
        Qhist.row(i) = q.coeffs().transpose();

        if (i < 50) {
            std::cout << "q: " << q << std::endl;
        }

        if (i < time.size()) {
            q0.x() = q_traj(i, 0);
            q0.y() = q_traj(i, 1);
            q0.z() = q_traj(i, 2);
            q0.w() = q_traj(i, 3);
            dmp.incrementalLearn(
                    phi(i), q0, omega_traj.row(i), alpha_traj.row(i)
            );
        }
        dmp.step();
    }

    std::ofstream   file("quaternion.csv");
    file << dmp::dumpToCsv(Qhist);
    file.close();

    return 0;
}
