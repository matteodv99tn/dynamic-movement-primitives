#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

#include "common/defines.hpp"
#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/quaternion_periodic_dmp.hpp"
#include "dmp/quaternion_utils.hpp"
#include "dmp/utils.hpp"
#include "gnuplot-iostream.h"

int main() {
    Eigen::MatrixXd trajectory = dmp::loadTrainingTrajectory();
    Eigen::MatrixXd q_traj     = dmp::getQuaternionTrajectory(trajectory);
    Eigen::MatrixXd omega_traj = dmp::getAngularVelocityTrajectory(trajectory);
    Eigen::MatrixXd alpha_traj = dmp::getAngularAccelerationTrajectory(trajectory);
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(50);

    dmp::QuaternionPeriodicDmp dmp(basis);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    Eigen::Vector4d    q0_elems = q_traj.row(0);
    Eigen::Quaterniond q0(q0_elems);
    dmp.setInitialConditions(q0, omega_traj.row(0));

    std::size_t        t_horizon = time.size() * 3;
    Eigen::MatrixXd    Qhist(t_horizon, 4);
    Eigen::Quaterniond q;

    dmp.batchLearn(phi, q_traj, omega_traj, alpha_traj);

    for (int i = 0; i < t_horizon; i++) {
        q = dmp.getQuaternionState();
        Qhist.row(i) = q.coeffs().transpose();
        dmp.step();
    }

    // Plotting
    Gnuplot gp;

    std::vector<double> qx     = dmp::test::toStdVector(Qhist.col(0));
    std::vector<double> qy     = dmp::test::toStdVector(Qhist.col(1));
    std::vector<double> qz     = dmp::test::toStdVector(Qhist.col(2));
    std::vector<double> qw     = dmp::test::toStdVector(Qhist.col(3));
    std::vector<double> qx_des = dmp::test::toStdVector(q_traj.col(0));
    std::vector<double> qy_des = dmp::test::toStdVector(q_traj.col(1));
    std::vector<double> qz_des = dmp::test::toStdVector(q_traj.col(2));
    std::vector<double> qw_des = dmp::test::toStdVector(q_traj.col(3));


    gp << "set title 'Quaternion DMP - Batch Learning'\n";
    gp << "set xlabel 'Time (ticks)'\n";
    gp << "set ylabel 'Quaternion'\n";
    gp << "plot '-' with lines title 'qx' linecolor 1"
          ", '-' with lines title 'qx des' dashtype 2 linecolor 1";
    gp << ", '-' with lines title 'qy' linecolor 2"
          ", '-' with lines title 'qy des' dashtype 2 linecolor 2";
    gp << ", '-' with lines title 'qz' linecolor 3"
          ", '-' with lines title 'qz des' dashtype 2 linecolor 3";
    gp << ", '-' with lines title 'qw' linecolor 4"
          ", '-' with lines title 'qw des' dashtype 2 linecolor 4";
    gp << "\n";

    gp.send1d(qx);
    gp.send1d(qx_des);
    gp.send1d(qy);
    gp.send1d(qy_des);
    gp.send1d(qz);
    gp.send1d(qz_des);
    gp.send1d(qw);
    gp.send1d(qw_des);


    return 0;
}
