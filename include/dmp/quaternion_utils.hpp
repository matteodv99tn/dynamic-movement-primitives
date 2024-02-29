#ifndef DMP__QUATERNION_UTILS_HPP__
#define DMP__QUATERNION_UTILS_HPP__

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dmp {

    Eigen::Vector3d logarithmic_map(const Eigen::Quaterniond& q);
    Eigen::Vector3d logarithmic_map(
            const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2
    );

    Eigen::Quaterniond exponential_map(const Eigen::Vector3d& v);
    Eigen::Quaterniond exponential_map(
            const Eigen::Vector3d& v, const Eigen::Quaterniond& q0
    );

}  // namespace dmp


#endif  // DMP__QUATERNION_UTILS_HPP__
