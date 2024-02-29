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
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(20);


    dmp::PeriodicDmp dmp(basis);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    Eigen::Vector2d x0(x_traj(0, 0), x_traj(0, 1));
    dmp.setInitialConditions(x0);

    std::size_t t_horizon = time.size() * 3;
    Eigen::MatrixXd Xhist(t_horizon, 3);

    for (int i = 0; i < t_horizon; i++) {
        Xhist.row(i) = dmp.getState().transpose();
        if (i < time.size()) {
            dmp.incrementalLearn(phi(i), x_traj.row(i));
        }
        dmp.step();
    }

    std::ofstream file("single_dmp.csv");
    file << dmp::dumpToCsv(Xhist);
    file.close();

    return 0;
}
