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

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(15);

    dmp::QuaternionPeriodicDmp dmp(basis);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    Eigen::Vector4d    q0_elems = q_traj.row(0);
    Eigen::Quaterniond q0(q0_elems);
    dmp.setInitialConditions(q0, omega_traj.row(0));

    std::size_t        t_horizon = time.size() * 3;
    Eigen::MatrixXd    Qhist(t_horizon, 4);
    Eigen::MatrixXd    Lhist(t_horizon, 3);
    Eigen::Quaterniond q;

    dmp.batchLearn(phi, q_traj, omega_traj, alpha_traj);

    for (int i = 0; i < t_horizon; i++) {
        q            = dmp.getQuaternionState();
        Qhist.row(i) = q.coeffs().transpose();
        Lhist.row(i) = dmp.getLogarithm();
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


    Gnuplot gp2;

    Eigen::MatrixXd     log_des  = dmp.computeLogarithms(q_traj);
    std::vector<double> logx     = dmp::test::toStdVector(Lhist.col(0));
    std::vector<double> logy     = dmp::test::toStdVector(Lhist.col(1));
    std::vector<double> logz     = dmp::test::toStdVector(Lhist.col(2));
    std::vector<double> logx_des = dmp::test::toStdVector(log_des.col(0));
    std::vector<double> logy_des = dmp::test::toStdVector(log_des.col(1));
    std::vector<double> logz_des = dmp::test::toStdVector(log_des.col(2));

    gp2 << "set title 'Quaternion DMP - Log. Map - Batch Learning'\n";
    gp2 << "set xlabel 'Time (ticks)'\n";
    gp2 << "set ylabel 'Quaternion'\n";
    gp2 << "plot '-' with lines title 'log_x' linecolor 1"
           ", '-' with lines title 'log_x des' dashtype 2 linecolor 1";
    gp2 << ", '-' with lines title 'log_y' linecolor 2"
           ", '-' with lines title 'log_y des' dashtype 2 linecolor 2";
    gp2 << ", '-' with lines title 'log_z' linecolor 3"
           ", '-' with lines title 'log_z des' dashtype 2 linecolor 3";
    gp2 << "\n";

    gp2.send1d(logx);
    gp2.send1d(logx_des);
    gp2.send1d(logy);
    gp2.send1d(logy_des);
    gp2.send1d(logz);
    gp2.send1d(logz_des);

    Gnuplot gp3;

    Eigen::MatrixXd rpy     = Eigen::MatrixXd::Zero(t_horizon, 3);
    Eigen::MatrixXd rpy_des = Eigen::MatrixXd::Zero(q_traj.rows(), 3);

    for (int i = 0; i < t_horizon; i++) {
        Eigen::Vector4d    q_elems = Qhist.row(i);
        Eigen::Quaterniond q       = Eigen::Quaterniond(q_elems);
        rpy.row(i) = q.toRotationMatrix().eulerAngles(0, 1, 2) * 180 / M_PI;
    }

    for (int i = 0; i < q_traj.rows(); i++) {
        Eigen::Vector4d    q_elems = q_traj.row(i);
        Eigen::Quaterniond q       = Eigen::Quaterniond(q_elems);
        rpy_des.row(i) = q.toRotationMatrix().eulerAngles(0, 1, 2) * 180 / M_PI;
    }


    std::vector<double> roll      = dmp::test::toStdVector(rpy.col(0));
    std::vector<double> pitch     = dmp::test::toStdVector(rpy.col(1));
    std::vector<double> yaw       = dmp::test::toStdVector(rpy.col(2));
    std::vector<double> roll_des  = dmp::test::toStdVector(rpy_des.col(0));
    std::vector<double> pitch_des = dmp::test::toStdVector(rpy_des.col(1));
    std::vector<double> yaw_des   = dmp::test::toStdVector(rpy_des.col(2));

    gp3 << "set title 'Quaternion DMP - RPY - Batch Learning'\n";
    gp3 << "set xlabel 'Time (ticks)'\n";
    gp3 << "set ylabel 'Quaternion'\n";
    gp3 << "plot '-' with lines title 'roll' linecolor 1"
           ", '-' with lines title 'roll des' dashtype 2 linecolor 1";
    gp3 << ", '-' with lines title 'pitch' linecolor 2"
           ", '-' with lines title 'pitch des' dashtype 2 linecolor 2";
    gp3 << ", '-' with lines title 'yaw' linecolor 3"
           ", '-' with lines title 'yaw des' dashtype 2 linecolor 3";
    gp3 << "\n";

    gp3.send1d(roll);
    gp3.send1d(roll_des);
    gp3.send1d(pitch);
    gp3.send1d(pitch_des);
    gp3.send1d(yaw);
    gp3.send1d(yaw_des);

    return 0;
}
