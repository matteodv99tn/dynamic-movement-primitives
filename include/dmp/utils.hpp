#ifndef DMP_UTILS_HPP__
#define DMP_UTILS_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <optional>
#include <string>

namespace dmp {
    namespace traindatacolumn {
        constexpr std::size_t time    = 0;
        constexpr std::size_t posx    = 1;
        constexpr std::size_t posy    = 2;
        constexpr std::size_t posz    = 3;
        constexpr std::size_t quatx   = 4;
        constexpr std::size_t quaty   = 5;
        constexpr std::size_t quatz   = 6;
        constexpr std::size_t quatw   = 7;
        constexpr std::size_t velx    = 8;
        constexpr std::size_t vely    = 9;
        constexpr std::size_t velz    = 10;
        constexpr std::size_t omegax  = 11;
        constexpr std::size_t omegay  = 12;
        constexpr std::size_t omegaz  = 13;
        constexpr std::size_t acclinx = 14;
        constexpr std::size_t accliny = 15;
        constexpr std::size_t acclinz = 16;
        constexpr std::size_t accrotx = 17;
        constexpr std::size_t accroty = 18;
        constexpr std::size_t accrotz = 19;
        constexpr std::size_t count   = 20;
    }  // namespace traindatacolumn

    Eigen::MatrixXd loadTrainingTrajectory();
    Eigen::MatrixXd loadTrainingTrajectory(const std::string& file_path);

    Eigen::VectorXd getTimeVector(const Eigen::MatrixXd& trajectory);
    std::optional<Eigen::MatrixXd> getAxisTrajectory(const Eigen::MatrixXd& trajectory, const char axis);

    Eigen::MatrixXd getPositionTrajectory(const Eigen::MatrixXd& trajectory);
    Eigen::MatrixXd getVelocityTrajectory(const Eigen::MatrixXd& trajectory);
    Eigen::MatrixXd getAccelerationTrajectory(const Eigen::MatrixXd& trajectory);

    Eigen::MatrixXd getQuaternionTrajectory(const Eigen::MatrixXd& data);
    Eigen::MatrixXd getAngularVelocityTrajectory(const Eigen::MatrixXd& data);
    Eigen::MatrixXd getAngularAccelerationTrajectory(const Eigen::MatrixXd& data);

    std::string dumpToCsv(const Eigen::MatrixXd& data);


}  // namespace dmp


#endif  // DMP_UTILS_HPP__
