#include "dmp/utils.hpp"

#include <cstddef>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <sys/stat.h>

#include "range/v3/all.hpp"

Eigen::MatrixXd dmp::loadTrainingTrajectory() {
    const std::string home_path = std::getenv("HOME");
    const std::string file_path = home_path + "/dmps/data/end_effector_states.csv";
    return dmp::loadTrainingTrajectory(file_path);
}

Eigen::MatrixXd dmp::loadTrainingTrajectory(const std::string& file_path) {
    const std::size_t num_cols = dmp::traindatacolumn::count;

    std::ifstream in(file_path);
    std::string   content(
            (std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>()
    );

    // Copy all the entries into a row-major std::vector that is then used to
    // initialize an Eigen::Matrix
    std::vector<double> elems = content | ranges::views::transform([](char c) {
                                    return ((c == ',') || (c == '\n')) ? ' ' : c;
                                }) |
                                ranges::views::split(' ') |
                                ranges::views::transform([](auto&& rng) -> double {
                                    return std::stod(rng | ranges::to<std::string>);
                                }) |
                                ranges::to<std::vector<double>>;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data(
            elems.size() / num_cols, num_cols
    );
    ranges::copy(elems, data.data());

    return data;
}

Eigen::VectorXd dmp::getTimeVector(const Eigen::MatrixXd& trajectory) {
    return trajectory.col(dmp::traindatacolumn::time);
}

std::optional<Eigen::MatrixXd> dmp::getAxisTrajectory(
        const Eigen::MatrixXd& trajectory, const char axis
) {
    std::size_t pos_idx, vel_idx, acc_idx;
    switch (axis) {
        case 'x':
            pos_idx = dmp::traindatacolumn::posx;
            vel_idx = dmp::traindatacolumn::velx;
            acc_idx = dmp::traindatacolumn::acclinx;
            break;
        case 'y':
            pos_idx = dmp::traindatacolumn::posy;
            vel_idx = dmp::traindatacolumn::vely;
            acc_idx = dmp::traindatacolumn::accliny;
            break;
        case 'z':
            pos_idx = dmp::traindatacolumn::posz;
            vel_idx = dmp::traindatacolumn::velz;
            acc_idx = dmp::traindatacolumn::acclinz;
            break;
        default:
            return std::nullopt;
    }

    Eigen::MatrixXd axis_trajectory(trajectory.rows(), 3);
    axis_trajectory.col(0) = trajectory.col(pos_idx);
    axis_trajectory.col(1) = trajectory.col(vel_idx);
    axis_trajectory.col(2) = trajectory.col(acc_idx);

    return axis_trajectory;
}

Eigen::MatrixXd dmp::getPositionTrajectory(const Eigen::MatrixXd& trajectory) {
    return trajectory.block(0, dmp::traindatacolumn::posx, trajectory.rows(), 3);
}

Eigen::MatrixXd dmp::getVelocityTrajectory(const Eigen::MatrixXd& trajectory) {
    return trajectory.block(0, dmp::traindatacolumn::velx, trajectory.rows(), 3);
}

Eigen::MatrixXd dmp::getAccelerationTrajectory(const Eigen::MatrixXd& trajectory) {
    return trajectory.block(0, dmp::traindatacolumn::acclinx, trajectory.rows(), 3);
}

Eigen::MatrixXd dmp::getQuaternionTrajectory(const Eigen::MatrixXd& data) {
    // return data.block(0, dmp::traindatacolumn::quatx, data.rows(), 4);
    Eigen::MatrixXd q(data.rows(), 4);
    q.col(0) = data.col(dmp::traindatacolumn::quatx);
    q.col(1) = data.col(dmp::traindatacolumn::quaty);
    q.col(2) = data.col(dmp::traindatacolumn::quatz);
    q.col(3) = data.col(dmp::traindatacolumn::quatw);

    const double th = 1e-15;
    for (Eigen::Index i = 0; i < data.rows(); i++) {
        // normalize the quaternion
        double norm_error = std::abs(q.row(i).norm() - 1);
        if (norm_error > th) {
            std::cout << "Quaternion at row " << i
                      << " is not normalized; norm error: " << norm_error << std::endl;
        }

        q.row(i).normalize();
    }
    return q;
}

Eigen::MatrixXd dmp::getAngularVelocityTrajectory(const Eigen::MatrixXd& data) {
    return data.block(0, dmp::traindatacolumn::omegax, data.rows(), 3);
}

Eigen::MatrixXd dmp::getAngularAccelerationTrajectory(const Eigen::MatrixXd& data) {
    return data.block(0, dmp::traindatacolumn::accrotx, data.rows(), 3);
}

std::string dmp::dumpToCsv(const Eigen::MatrixXd& data) {
    std::string res = "";

    for (int i = 0; i < data.rows(); i++) {
        for (int j = 0; j < data.cols(); j++) {
            res += std::to_string(data(i, j));
            if (j < data.cols() - 1) res += ",";
        }
        res += "\n";
    }
    return res;
}

Eigen::MatrixXd dmp::finiteDifference(const Eigen::MatrixXd& data, const double dt) {
    const std::size_t rows = data.rows();
    const std::size_t cols = data.cols();
    Eigen::MatrixXd   diff(rows, cols);

    diff.block(0, 0, rows - 1, cols) =
            (data.block(1, 0, rows - 1, cols) - data.block(0, 0, rows - 1, cols)) / dt;
    diff.row(rows - 1) = diff.row(0);
    return diff;
}
