#include <Eigen/Dense>
#include <cstddef>
#include <iostream>
#include <fstream>

#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/multidof_periodic_dmp.hpp"
#include "dmp/utils.hpp"
#include "common/defines.hpp"
#include "gnuplot-iostream.h"

int main() {
    Eigen::MatrixXd trajectory = dmp::loadTrainingTrajectory(dmp::test::csv_path);
    Eigen::MatrixXd pos_traj     = dmp::getPositionTrajectory(trajectory);
    Eigen::MatrixXd vel_traj     = dmp::getVelocityTrajectory(trajectory);
    Eigen::MatrixXd acc_traj     = dmp::getAccelerationTrajectory(trajectory);
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(20);

    dmp::MultiDofPeriodicDmp dmp(basis, 3);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    dmp.setInitialConditions(
        pos_traj.row(0),
        vel_traj.row(0)
    );

    std::size_t t_horizon = time.size() * 3;
    Eigen::MatrixXd Hist(t_horizon, 9);
    dmp.batchLearn(phi, pos_traj, vel_traj, acc_traj);

    for (int i = 0; i < t_horizon; i++) {
        Hist.row(i).head(3) = dmp.getPositionState().transpose();
        Hist.row(i).segment(3, 3) = dmp.getVelocityState().transpose();
        Hist.row(i).tail(3) = dmp.getAccelerationState().transpose();

        // if (i < time.size()) {
        //     x_dmp.incrementalLearn(phi(i), x_traj.row(i));
        //     y_dmp.incrementalLearn(phi(i), y_traj.row(i));
        //     z_dmp.incrementalLearn(phi(i), z_traj.row(i));
        // }
        dmp.step();
    }

    std::ofstream file(dmp::test::results_directory + "/3dof_dmp_batch.csv");
    file << "x,y,z,vx,vy,vz,ax,ay,az\n";
    file << dmp::dumpToCsv(Hist);
    file.close();

    // Plotting
    Gnuplot gp;

    std::vector<double> xvec = dmp::test::toStdVector(Hist.col(0));
    std::vector<double> yvec = dmp::test::toStdVector(Hist.col(1));
    std::vector<double> zvec = dmp::test::toStdVector(Hist.col(2));
    std::vector<double> xvec_des = dmp::test::toStdVector(pos_traj.col(0));
    std::vector<double> yvec_des = dmp::test::toStdVector(pos_traj.col(1));
    std::vector<double> zvec_des = dmp::test::toStdVector(pos_traj.col(2));


    gp << "set title '3D Cartesian DMP - Batch Learning'\n";
    gp << "set xlabel 'Time (ticks)'\n";
    gp << "set ylabel 'Cartesian position'\n";
    gp << "plot '-' with lines title 'x', '-' with lines title 'x des' dashtype 2";
    gp << ", '-' with lines title 'y', '-' with lines title 'y des' dashtype 2";
    gp << ", '-' with lines title 'z', '-' with lines title 'z des' dashtype 2";
    gp << "\n";

    gp.send1d(xvec);
    gp.send1d(xvec_des);
    gp.send1d(yvec);
    gp.send1d(yvec_des);
    gp.send1d(zvec);
    gp.send1d(zvec_des);

    return 0;
}
