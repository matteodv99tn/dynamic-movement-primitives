#include "test_functions.hpp"

#include "defines.hpp"
#include "dmp/basis_function/periodic_gaussian_kernel.hpp"
#include "dmp/multidof_periodic_dmp.hpp"
#include "dmp/quaternion_periodic_dmp.hpp"
#include "dmp/quaternion_utils.hpp"
#include "dmp/utils.hpp"
#include "gnuplot-iostream.h"

Eigen::MatrixXd dmp::test::load_file(const std::string& file_name) {
    std::string file_path = dmp::test::data_directory + "/" + file_name;
    return dmp::loadTrainingTrajectory(file_path);
}

void dmp::test::batch_learning_test(
        const Eigen::MatrixXd& data,
        const double           alpha,
        const std::size_t      N_basis,
        bool                   rotate_omega,
        const std::size_t      N_reps,
        const dmp::test::LearningMethod    method
) {
    // Extract data
    Eigen::VectorXd time                = dmp::getTimeVector(data);
    Eigen::MatrixXd pos_traj            = dmp::getPositionTrajectory(data);
    Eigen::MatrixXd vel_traj            = dmp::getVelocityTrajectory(data);
    Eigen::MatrixXd acc_traj            = dmp::getAccelerationTrajectory(data);
    Eigen::MatrixXd q_traj              = dmp::getQuaternionTrajectory(data);
    Eigen::MatrixXd omega_traj_original = dmp::getAngularVelocityTrajectory(data);
    Eigen::MatrixXd alpha_traj_original = dmp::getAngularAccelerationTrajectory(data);
    Eigen::MatrixXd omega_traj, alpha_traj;

    // Process data (if needed)
    if (rotate_omega) {
        omega_traj = dmp::rotate_angular_velocity(omega_traj_original, q_traj);
        alpha_traj = dmp::rotate_angular_velocity(alpha_traj_original, q_traj);
    } else {
        omega_traj = omega_traj_original;
        alpha_traj = alpha_traj_original;
    }


    const std::size_t Ns       = time.size();        // Number of samples
    const double      dt       = time(1) - time(0);  // Sampling period
    const double      T_period = time(Ns - 1);       // Period of the trajectory

    omega_traj = dmp::quaternion_numerical_diff(q_traj, dt);
    alpha_traj = dmp::finiteDifference(omega_traj, dt);


    std::string sep = "=============";
    std::cout << sep << " Test Settings " << sep << "\n";
    if(method == dmp::test::LearningMethod::BATCH)
        std::cout << "Learning method:       batch\n";
    else
        std::cout << "Learning method:       incremental\n";
    std::cout << "Alpha:                 " << alpha << "\n";
    std::cout << "Num. of basis:         " << N_basis << "\n";
    std::cout << "Sampling period:       " << dt << "\n";
    std::cout << "Demonstration samples: " << Ns << "\n";
    std::cout << "Demonstration period:  " << T_period << "\n";
    std::cout << "Simulation cycles:     " << N_reps << "\n";
    std::cout << "Rotate omega?          " << std::boolalpha << rotate_omega << "\n";


    // Create DMP
    auto basis = std::make_shared<dmp::PeriodicGaussianKernel>(N_basis);
    dmp::MultiDofPeriodicDmp   dmp_position(basis, 3, alpha);
    dmp::QuaternionPeriodicDmp dmp_orientation(basis, alpha);

    // Set properties
    dmp_position.setInitialConditions(pos_traj.row(0), vel_traj.row(0));
    dmp_position.setSamplingPeriod(dt);
    dmp_position.setObservationPeriod(T_period);

    Eigen::Quaterniond q0(Eigen::Vector4d(q_traj.row(0)));
    dmp_orientation.setInitialConditions(q0, omega_traj.row(0));
    dmp_orientation.setSamplingPeriod(dt);
    dmp_orientation.setObservationPeriod(T_period);

    std::cout << "Tau:   " << dmp_orientation.getTau() << "\n";
    std::cout << "Omega: " << 1 / dmp_orientation.getTau() << "\n";


    // Prepare for storage
    std::size_t        t_sim = Ns * N_reps;
    Eigen::MatrixXd    pos_hist(t_sim, 3);
    Eigen::MatrixXd    vel_hist(t_sim, 3);
    Eigen::MatrixXd    acc_hist(t_sim, 3);
    Eigen::MatrixXd    q_hist(t_sim, 4);
    Eigen::MatrixXd    log_hist(t_sim, 3);
    Eigen::MatrixXd    omega_hist(t_sim, 3);
    Eigen::MatrixXd    alpha_hist(t_sim, 3);
    Eigen::Quaterniond q;

    // Perform batch learning
    Eigen::VectorXd phi_position = dmp_position.timeToPhase(time);
    Eigen::VectorXd phi_orientation = dmp_orientation.timeToPhase(time);
    if (method == dmp::test::LearningMethod::BATCH) {
        dmp_position.batchLearn(phi_position, pos_traj, vel_traj, acc_traj);
        dmp_orientation.batchLearn(phi_orientation, q_traj, omega_traj, alpha_traj);
    }

    // Integrate the system
    for (int i = 0; i < t_sim; i++) {

        // Store states
        pos_hist.row(i) = dmp_position.getPositionState();
        vel_hist.row(i) = dmp_position.getVelocityState();
        acc_hist.row(i) = dmp_position.getAccelerationState();

        q                 = dmp_orientation.getQuaternionState();
        q_hist.row(i)     = q.coeffs();
        log_hist.row(i)   = dmp_orientation.getLogarithm();
        omega_hist.row(i) = dmp_orientation.getAngularVelocityState();
        alpha_hist.row(i) = dmp_orientation.getAngularAcceleration();

        // Incremental learn
        if ((method == dmp::test::LearningMethod::INCREMENTAL) && (i < Ns)) {
            Eigen::Quaterniond q_tmp(Eigen::Vector4d(q_traj.row(i)));
            dmp_position.incrementalLearn(phi_position(i), pos_traj.row(i), vel_traj.row(i), acc_traj.row(i));
            dmp_orientation.incrementalLearn(phi_orientation(i), q_tmp, omega_traj.row(i), alpha_traj.row(i));
        }

        // Step integration
        dmp_orientation.step();
        dmp_position.step();
    }

    Eigen::MatrixXd diff = q_traj - q_hist.block(0, 0, Ns, 4);

    Eigen::MatrixXd pos_traj_repeated = Eigen::MatrixXd::Zero(t_sim, 3);
    Eigen::MatrixXd q_traj_repeated = Eigen::MatrixXd::Zero(t_sim, 4);

    for (int i = 0; i < N_reps; i++) {
        pos_traj_repeated.block(i * Ns, 0, Ns, 3) = pos_traj;
        q_traj_repeated.block(i * Ns, 0, Ns, 4) = q_traj;
    }

    // Plot results
    dmp::test::plot_quaternion_trajectory(q_hist, q_traj_repeated);
    dmp::test::plot_position_trajectory(pos_hist, pos_traj_repeated);
    // dmp::test::plot_angular_velocity(omega_hist, omega_traj);
    // dmp::test::plot_angular_acceleration(alpha_hist, alpha_traj);
    // dmp::test::plot_forcing_term(
    //         dmp_orientation.getLearnedForcingFunction(phi_orientation),
    //         dmp_orientation.evaluateDesiredForce(q_traj, omega_traj, alpha_traj)
    // );
}

void dmp::test::plot_quaternion_trajectory(
        const Eigen::MatrixXd& q_sim, const Eigen::MatrixXd& q_des
) {
    std::vector<double> qx     = dmp::test::toStdVector(q_sim.col(0));
    std::vector<double> qy     = dmp::test::toStdVector(q_sim.col(1));
    std::vector<double> qz     = dmp::test::toStdVector(q_sim.col(2));
    std::vector<double> qw     = dmp::test::toStdVector(q_sim.col(3));
    std::vector<double> qx_des = dmp::test::toStdVector(q_des.col(0));
    std::vector<double> qy_des = dmp::test::toStdVector(q_des.col(1));
    std::vector<double> qz_des = dmp::test::toStdVector(q_des.col(2));
    std::vector<double> qw_des = dmp::test::toStdVector(q_des.col(3));

    Gnuplot gp;
    gp << "set title 'Quaternions Trajectory'\n";
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
}

void dmp::test::plot_position_trajectory(
        const Eigen::MatrixXd& x_sim, const Eigen::MatrixXd& x_des
) {
    std::vector<double> x  = dmp::test::toStdVector(x_sim.col(0));
    std::vector<double> y  = dmp::test::toStdVector(x_sim.col(1));
    std::vector<double> z  = dmp::test::toStdVector(x_sim.col(2));
    std::vector<double> xd = dmp::test::toStdVector(x_des.col(0));
    std::vector<double> yd = dmp::test::toStdVector(x_des.col(1));
    std::vector<double> zd = dmp::test::toStdVector(x_des.col(2));

    Gnuplot gp;
    gp << "set title 'Cartesian Trajectory'\n";
    gp << "set xlabel 'Time (ticks)'\n";
    gp << "set ylabel 'Position'\n";
    gp << "plot '-' with lines title 'x' linecolor 1"
          ", '-' with lines title 'x des' dashtype 2 linecolor 1";
    gp << ", '-' with lines title 'y' linecolor 2"
          ", '-' with lines title 'y des' dashtype 2 linecolor 2";
    gp << ", '-' with lines title 'z' linecolor 3"
          ", '-' with lines title 'z des' dashtype 2 linecolor 3";
    gp << "\n";

    gp.send1d(x);
    gp.send1d(xd);
    gp.send1d(y);
    gp.send1d(yd);
    gp.send1d(z);
    gp.send1d(zd);
}

void dmp::test::plot_forcing_term(
        const Eigen::MatrixXd& F_learned, const Eigen::MatrixXd& F_desired
) {
    std::vector<double> Fx_lrn = dmp::test::toStdVector(F_learned.col(0));
    std::vector<double> Fy_lrn = dmp::test::toStdVector(F_learned.col(1));
    std::vector<double> Fz_lrn = dmp::test::toStdVector(F_learned.col(2));
    std::vector<double> Fx_des = dmp::test::toStdVector(F_desired.col(0));
    std::vector<double> Fy_des = dmp::test::toStdVector(F_desired.col(1));
    std::vector<double> Fz_des = dmp::test::toStdVector(F_desired.col(2));
    std::vector<double> Fw_des = dmp::test::toStdVector(F_desired.col(3));

    Gnuplot gp;
    gp << "set title 'Forcing terms'\n";
    gp << "set xlabel 'Time (ticks)'\n";
    gp << "set ylabel 'Value'\n";
    gp << "plot '-' with lines title 'F_x lrn' linecolor 1"
          ", '-' with lines title 'F_x des' dashtype 2 linecolor 1";
    gp << ", '-' with lines title 'F_y lrn' linecolor 2"
          ", '-' with lines title 'F_y des' dashtype 2 linecolor 2";
    gp << ", '-' with lines title 'F_z lrn' linecolor 3"
          ", '-' with lines title 'F_z des' dashtype 2 linecolor 3";
    gp << "\n";

    gp.send1d(Fx_lrn);
    gp.send1d(Fx_des);
    gp.send1d(Fy_lrn);
    gp.send1d(Fy_des);
    gp.send1d(Fz_lrn);
    gp.send1d(Fz_des);
}

void dmp::test::plot_angular_velocity(
        const Eigen::MatrixXd& omega_input, const Eigen::MatrixXd& omega_generated
) {
    std::vector<double> wx_in  = dmp::test::toStdVector(omega_input.col(0));
    std::vector<double> wy_in  = dmp::test::toStdVector(omega_input.col(1));
    std::vector<double> wz_in  = dmp::test::toStdVector(omega_input.col(2));
    std::vector<double> wx_gen = dmp::test::toStdVector(omega_generated.col(0));
    std::vector<double> wy_gen = dmp::test::toStdVector(omega_generated.col(1));
    std::vector<double> wz_gen = dmp::test::toStdVector(omega_generated.col(2));

    Gnuplot gp;
    gp << "set title 'Angular velocity'\n";
    gp << "set xlabel 'Time (ticks)'\n";
    gp << "set ylabel 'rad/s'\n";
    gp << "plot '-' with lines title '{/Symbol w}_x gen' linecolor 1";
    gp << ", '-' with lines title    '{/Symbol w}_x data' linecolor 1 dashtype 2";
    gp << ", '-' with lines title    '{/Symbol w}_y gen' linecolor 2";
    gp << ", '-' with lines title    '{/Symbol w}_y data' linecolor 2 dashtype 2";
    gp << ", '-' with lines title    '{/Symbol w}_z gen' linecolor 3";
    gp << ", '-' with lines title    '{/Symbol w}_z data' linecolor 3 dashtype 2";
    gp << "\n";

    gp.send1d(wx_in);
    gp.send1d(wx_gen);
    gp.send1d(wy_in);
    gp.send1d(wy_gen);
    gp.send1d(wz_in);
    gp.send1d(wz_gen);
}

void dmp::test::plot_angular_acceleration(
        const Eigen::MatrixXd& alpha_input, const Eigen::MatrixXd& alpha_generated
) {
    std::vector<double> wx_in  = dmp::test::toStdVector(alpha_input.col(0));
    std::vector<double> wy_in  = dmp::test::toStdVector(alpha_input.col(1));
    std::vector<double> wz_in  = dmp::test::toStdVector(alpha_input.col(2));
    std::vector<double> wx_gen = dmp::test::toStdVector(alpha_generated.col(0));
    std::vector<double> wy_gen = dmp::test::toStdVector(alpha_generated.col(1));
    std::vector<double> wz_gen = dmp::test::toStdVector(alpha_generated.col(2));

    Gnuplot gp;
    gp << "set title 'Angular acceleration'\n";
    gp << "set xlabel 'Time (ticks)'\n";
    gp << "set ylabel 'rad/s^2'\n";
    gp << "plot '-' with lines title '{/Symbol a}_x gen' linecolor 1";
    gp << ", '-' with lines title    '{/Symbol a}_x data' linecolor 1 dashtype 2";
    gp << ", '-' with lines title    '{/Symbol a}_y gen' linecolor 2";
    gp << ", '-' with lines title    '{/Symbol a}_y data' linecolor 2 dashtype 2";
    gp << ", '-' with lines title    '{/Symbol a}_z gen' linecolor 3";
    gp << ", '-' with lines title    '{/Symbol a}_z data' linecolor 3 dashtype 2";
    gp << "\n";

    gp.send1d(wx_in);
    gp.send1d(wx_gen);
    gp.send1d(wy_in);
    gp.send1d(wy_gen);
    gp.send1d(wz_in);
    gp.send1d(wz_gen);
}

std::vector<double> dmp::test::toStdVector(const Eigen::VectorXd& vec) {
    std::vector<double> std_vec(vec.rows());
    for (int i = 0; i < vec.size(); i++) { std_vec[i] = vec(i); }
    return std_vec;
}
