#include <cstddef>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/multidof_periodic_dmp.hpp"
#include "dmp/utils.hpp"

int main() {
    Eigen::MatrixXd trajectory = dmp::loadTrainingTrajectory();
    Eigen::MatrixXd pos_traj   = dmp::getPositionTrajectory(trajectory);
    Eigen::MatrixXd vel_traj   = dmp::getVelocityTrajectory(trajectory);
    Eigen::MatrixXd acc_traj   = dmp::getAccelerationTrajectory(trajectory);
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(20);

    dmp::MultiDofPeriodicDmp dmp(basis, 3);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    dmp.setInitialConditions(pos_traj.row(0), vel_traj.row(0));

    std::size_t     t_horizon = time.size() * 3;
    Eigen::MatrixXd Xhist(t_horizon, 3);
    Eigen::MatrixXd Yhist(t_horizon, 3);
    Eigen::MatrixXd Zhist(t_horizon, 3);
    Eigen::Vector3d pos, vel, acc;

    dmp.batchLearn(phi, pos_traj, vel_traj, acc_traj);
    for (int i = 0; i < t_horizon; i++) {
        Eigen::VectorXd pos = dmp.getPositionState();
        Eigen::VectorXd vel = dmp.getVelocityState();
        Eigen::VectorXd acc = dmp.getAccelerationState();

        Xhist.row(i) << pos(0), vel(0), acc(0);
        Yhist.row(i) << pos(1), vel(1), acc(1);
        Zhist.row(i) << pos(2), vel(2), acc(2);

        // if (i < time.size()) {
        //     dmp.incrementalLearn(
        //             phi(i), pos_traj.row(i), vel_traj.row(i), acc_traj.row(i)
        //     );
        // }
        dmp.step();
    }

    std::ofstream   file("multiple_dmp.csv");
    Eigen::MatrixXd hist = Eigen::MatrixXd::Zero(t_horizon, 9);
    hist << Xhist, Yhist, Zhist;
    file << dmp::dumpToCsv(hist);
    file.close();

    return 0;
}
