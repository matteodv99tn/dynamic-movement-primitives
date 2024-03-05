#include <cstddef>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>

#include "common/defines.hpp"
#include "common/defines.hpp.in"
#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/quaternion_periodic_dmp.hpp"
#include "dmp/quaternion_utils.hpp"
#include "dmp/utils.hpp"
#include "gnuplot-iostream.h"

int main() {
    Eigen::MatrixXd trajectory =
            dmp::loadTrainingTrajectory(dmp::test::data_directory + "/synthetic.csv");
    Eigen::MatrixXd q_traj     = dmp::getQuaternionTrajectory(trajectory);
    Eigen::MatrixXd omega_traj = dmp::getAngularVelocityTrajectory(trajectory);
    Eigen::MatrixXd alpha_traj = dmp::getAngularAccelerationTrajectory(trajectory);
    Eigen::VectorXd time       = dmp::getTimeVector(trajectory);

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(30);


    dmp::QuaternionPeriodicDmp dmp(basis, 48.0, 0.9999);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);
    double          dt  = 0.002;
    dmp.setSamplingPeriod(dt);

    omega_traj = dmp::quaternion_numerical_diff(q_traj, dt);
    alpha_traj = dmp::finiteDifference(omega_traj, dt);
    Gnuplot gp_quat;  // Quaternions

    std::vector<double> qx_des = dmp::test::toStdVector(q_traj.col(0));
    std::vector<double> qy_des = dmp::test::toStdVector(q_traj.col(1));
    std::vector<double> qz_des = dmp::test::toStdVector(q_traj.col(2));
    std::vector<double> qw_des = dmp::test::toStdVector(q_traj.col(3));


    gp_quat << "set title 'Quaternion Trajectory'\n";
    gp_quat << "set xlabel 'Time (ticks)'\n";
    gp_quat << "set ylabel 'Quaternion'\n";
    gp_quat << "plot '-' with lines title 'qx' linecolor 1";
    gp_quat << ", '-' with lines title 'qy' linecolor 2";
    gp_quat << ", '-' with lines title 'qz' linecolor 3";
    gp_quat << ", '-' with lines title 'qw' linecolor 4";
    gp_quat << "\n";

    gp_quat.send1d(qx_des);
    gp_quat.send1d(qy_des);
    gp_quat.send1d(qz_des);
    gp_quat.send1d(qw_des);

    Gnuplot gp_omega;  // Angular velocity comparison

    Eigen::MatrixXd omega_original = dmp::getAngularVelocityTrajectory(trajectory);
    for (std::size_t i = 0; i < omega_original.rows(); ++i) {
        Eigen::Quaterniond qrot(Eigen::Vector4d(q_traj.row(i)));
        omega_original.row(i) = qrot * omega_original.row(i);
    }

    std::vector<double> omegadiff_x = dmp::test::toStdVector(omega_traj.col(0));
    std::vector<double> omegadiff_y = dmp::test::toStdVector(omega_traj.col(1));
    std::vector<double> omegadiff_z = dmp::test::toStdVector(omega_traj.col(2));
    std::vector<double> omegajac_x  = dmp::test::toStdVector(omega_original.col(0));
    std::vector<double> omegajac_y  = dmp::test::toStdVector(omega_original.col(1));
    std::vector<double> omegajac_z  = dmp::test::toStdVector(omega_original.col(2));

    gp_omega << "set title 'Angular velocity comparison\n";
    gp_omega << "set xlabel 'Time (ticks)'\n";
    gp_omega << "set ylabel 'Angular velocity'\n";
    gp_omega << "plot '-' with lines title '{/Symbol w}_x diff' linecolor 1"
                ", '-' with lines title '{/Symbol w}_x data' dashtype 2 linecolor 1";
    gp_omega << ", '-' with lines title '{/Symbol w}_y diff' linecolor 2"
                ", '-' with lines title '{/Symbol w}_y data' dashtype 2 linecolor 2";
    gp_omega << ", '-' with lines title '{/Symbol w}_z diff' linecolor 3"
                ", '-' with lines title '{/Symbol w}_z data' dashtype 2 linecolor 3";
    gp_omega << "\n";

    gp_omega.send1d(omegadiff_x);
    gp_omega.send1d(omegajac_x);
    gp_omega.send1d(omegadiff_y);
    gp_omega.send1d(omegajac_y);
    gp_omega.send1d(omegadiff_z);
    gp_omega.send1d(omegajac_z);

    return 0;
}
