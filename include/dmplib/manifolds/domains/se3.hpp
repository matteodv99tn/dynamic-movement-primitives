#ifndef DMPLIB_SE3_DOMAIN_HPP
#define DMPLIB_SE3_DOMAIN_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace dmp{

struct SE3 { // NOLINT for struct name
    using Vec3_t = Eigen::Vector3d;
    using Quat_t = Eigen::Quaterniond;

    Vec3_t p;
    Quat_t q;

    SE3(Vec3_t position = Vec3_t::Zero(), Quat_t orientation = Quat_t::Identity());
};

}



#endif  // DMPLIB_SE3_DOMAIN_HPP
