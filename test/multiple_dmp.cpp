#include <Eigen/Dense>
#include <cstddef>
#include <iostream>
#include <fstream>

#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/periodic_dmp.hpp"
#include "dmp/utils.hpp"

int main() {
    Eigen::MatrixXd trajectory = dmp::loadTrainingTrajectory();
    Eigen::MatrixXd x_traj     = dmp::getAxisTrajectory(trajectory, 'x').value();
    Eigen::MatrixXd y_traj     = dmp::getAxisTrajectory(trajectory, 'y').value();
    Eigen::MatrixXd z_traj     = dmp::getAxisTrajectory(trajectory, 'z').value();
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(20);


    dmp::PeriodicDmp x_dmp(basis);
    dmp::PeriodicDmp y_dmp(basis);
    dmp::PeriodicDmp z_dmp(basis);
    x_dmp.setObservationPeriod(time(time.size() - 1));
    y_dmp.setObservationPeriod(time(time.size() - 1));
    z_dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = x_dmp.timeToPhase(time);

    Eigen::Vector2d x0(x_traj(0, 0), x_traj(0, 1));
    Eigen::Vector2d y0(y_traj(0, 0), y_traj(0, 1));
    Eigen::Vector2d z0(z_traj(0, 0), z_traj(0, 1));
    x_dmp.setInitialConditions(x0);
    y_dmp.setInitialConditions(y0);
    z_dmp.setInitialConditions(z0);

    std::size_t t_horizon = time.size() * 3;
    Eigen::MatrixXd Xhist(t_horizon, 3);
    Eigen::MatrixXd Yhist(t_horizon, 3);
    Eigen::MatrixXd Zhist(t_horizon, 3);

    for (int i = 0; i < t_horizon; i++) {
        Xhist.row(i) = x_dmp.getState().transpose();
        Yhist.row(i) = y_dmp.getState().transpose();
        Zhist.row(i) = z_dmp.getState().transpose();
        if (i < time.size()) {
            x_dmp.incrementalLearn(phi(i), x_traj.row(i));
            y_dmp.incrementalLearn(phi(i), y_traj.row(i));
            z_dmp.incrementalLearn(phi(i), z_traj.row(i));
        }
        x_dmp.step();
        y_dmp.step();
        z_dmp.step();
    }

    std::ofstream file("multiple_dmp.csv");
    Eigen::MatrixXd hist = Eigen::MatrixXd::Zero(t_horizon, 9);
    hist << Xhist, Yhist, Zhist;
    file << dmp::dumpToCsv(hist);
    file.close();

    return 0;
}
