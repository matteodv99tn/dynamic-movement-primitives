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

    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(20);


    dmp::QuaternionPeriodicDmp dmp(basis);
    dmp.setObservationPeriod(time(time.size() - 1));
    Eigen::VectorXd phi = dmp.timeToPhase(time);

    Eigen::MatrixXd q_ts = dmp.computeLogarithms(q_traj);
    omega_traj.block(0, 0, q_ts.rows() - 1, 3) =
            (q_ts.block(1, 0, q_ts.rows(), 3) - q_ts.block(0, 0, q_ts.rows() - 1, 3)) /
            0.002;
    omega_traj.row(q_ts.rows() - 1) = omega_traj.row(0);

    alpha_traj.block(0, 0, omega_traj.rows() - 1, 3) =
            (omega_traj.block(1, 0, omega_traj.rows(), 3) -
             omega_traj.block(0, 0, omega_traj.rows() - 1, 3)) /
            0.002;
    alpha_traj.row(q_ts.rows() - 1) = alpha_traj.row(0);


    Eigen::Vector4d    q0_elems = q_traj.row(0);
    Eigen::Quaterniond q0(q0_elems);
    dmp.setInitialConditions(q0, omega_traj.row(0));

    std::size_t        t_horizon = time.size() * 4;
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


#if 0
    Gnuplot gp2; // Quaternion logarithms

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
#endif

#if 0
    Gnuplot gp3; // RPY

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
#endif

#if 0
    Gnuplot gp4; // Angular acceleration

    std::vector<double> alpha_x      = dmp::test::toStdVector(alpha_traj.col(0));
    std::vector<double> alpha_y      = dmp::test::toStdVector(alpha_traj.col(1));
    std::vector<double> alpha_z      = dmp::test::toStdVector(alpha_traj.col(2));

    gp4 << "set title 'Quaternion DMP - Angular acceleration - Batch Learning'\n";
    gp4 << "set xlabel 'Time (ticks)'\n";
    gp4 << "set ylabel 'Quaternion'\n";
    gp4 << "plot '-' with lines title 'domega_x' linecolor 1";
    gp4 << ", '-' with lines title 'domega_y' linecolor 2";
    gp4 << ", '-' with lines title 'domega_z' linecolor 3";
    gp4 << "\n";

    gp4.send1d(alpha_x);
    gp4.send1d(alpha_y);
    gp4.send1d(alpha_z);
#endif

#if 0
    Gnuplot gp5; // Learned forcing terms

    Eigen::MatrixXd f_learned = dmp.getLearnedForcingFunction(phi);
    Eigen::MatrixXd f_desired =
            dmp.evaluateDesiredForce(q_traj, omega_traj, alpha_traj);

    std::vector<double> f_learned_x = dmp::test::toStdVector(f_learned.col(0));
    std::vector<double> f_learned_y = dmp::test::toStdVector(f_learned.col(1));
    std::vector<double> f_learned_z = dmp::test::toStdVector(f_learned.col(2));
    std::vector<double> f_desired_x = dmp::test::toStdVector(f_desired.col(0));
    std::vector<double> f_desired_y = dmp::test::toStdVector(f_desired.col(1));
    std::vector<double> f_desired_z = dmp::test::toStdVector(f_desired.col(2));

    gp5 << "set title 'Quaternion DMP - Forcing terms - Batch Learning'\n";
    gp5 << "set xlabel 'Time (ticks)'\n";
    gp5 << "set ylabel 'Quaternion'\n";
    gp5 << "plot '-' with lines title 'f_x learned' linecolor 1"
           ", '-' with lines title 'f_x desired' dashtype 2 linecolor 1";
    gp5 << ", '-' with lines title 'f_y learned' linecolor 2"
           ", '-' with lines title 'f_y desired' dashtype 2 linecolor 2";
    gp5 << ", '-' with lines title 'f_z learned' linecolor 3"
           ", '-' with lines title 'f_z desired' dashtype 2 linecolor 3";
    gp5 << "\n";

    gp5.send1d(f_learned_x);
    gp5.send1d(f_desired_x);
    gp5.send1d(f_learned_y);
    gp5.send1d(f_desired_y);
    gp5.send1d(f_learned_z);
    gp5.send1d(f_desired_z);
#endif

    Gnuplot gp6;

    Eigen::MatrixXd omega_original = dmp::getAngularVelocityTrajectory(trajectory);

    std::vector<double> omegadiff_x = dmp::test::toStdVector(omega_traj.col(0));
    std::vector<double> omegadiff_y = dmp::test::toStdVector(omega_traj.col(1));
    std::vector<double> omegadiff_z = dmp::test::toStdVector(omega_traj.col(2));
    std::vector<double> omegajac_x  = dmp::test::toStdVector(omega_original.col(0));
    std::vector<double> omegajac_y  = dmp::test::toStdVector(omega_original.col(1));
    std::vector<double> omegajac_z  = dmp::test::toStdVector(omega_original.col(2));

    gp6 << "set title 'Quaternion DMP - Angula velocity - Batch Learning'\n";
    gp6 << "set xlabel 'Time (ticks)'\n";
    gp6 << "set ylabel 'Quaternion'\n";
    gp6 << "plot '-' with lines title 'omega x diff' linecolor 1"
           ", '-' with lines title 'omega x jacobian' dashtype 2 linecolor 1";
    gp6 << ", '-' with lines title 'omega y diff' linecolor 2"
           ", '-' with lines title 'omega y jacobian' dashtype 2 linecolor 2";
    gp6 << ", '-' with lines title 'omega z diff' linecolor 3"
           ", '-' with lines title 'omega z jacobian' dashtype 2 linecolor 3";
    gp6 << "\n";

    gp6.send1d(omegadiff_x);
    gp6.send1d(omegajac_x);
    gp6.send1d(omegadiff_y);
    gp6.send1d(omegajac_y);
    gp6.send1d(omegadiff_z);
    gp6.send1d(omegajac_z);

    return 0;
}
