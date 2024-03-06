#ifndef DMP_TEST_FUNCTIONS_HPP__
#define DMP_TEST_FUNCTIONS_HPP__

#include <Eigen/Dense>
#include <string>

namespace dmp::test {

    // Filename inside the test/data/ directory
    Eigen::MatrixXd load_file(const std::string& file_name);

    void batch_learning_test(
            const Eigen::MatrixXd& data,
            const double           alpha,
            const std::size_t      N_basis,
            bool                   rotate_omega,
            const std::size_t      N_reps
    );

    void plot_quaternion_trajectory(
            const Eigen::MatrixXd& q_sim, const Eigen::MatrixXd& q_des
    );

    void plot_forcing_term(
            const Eigen::MatrixXd& F_learned, const Eigen::MatrixXd& F_desired
    );

    void plot_angular_velocity(
            const Eigen::MatrixXd& omega_input, const Eigen::MatrixXd& omega_generated
    );
    void plot_angular_acceleration(
            const Eigen::MatrixXd& alpha_input, const Eigen::MatrixXd& alpha_generated
    );

    std::vector<double> toStdVector(const Eigen::VectorXd& vec);

}  // namespace dmp::test

#endif  // DMP_TEST_FUNCTIONS_HPP__
