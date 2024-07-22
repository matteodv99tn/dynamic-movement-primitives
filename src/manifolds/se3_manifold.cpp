#include "dmplib/manifolds/se3_manifold.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"

namespace rm = dmp::riemannmanifold;

using rm::SE3;
using rm::Vec3_t;
using rm::Vec6_t;

SE3::SE3(Vec3_t position, Quaternion_t orientation) :
        pos(std::move(position)), ori(std::move(orientation)) {};

SE3::SE3(const Eigen::Matrix4d& homog_transform) :
        pos(homog_transform.topRightCorner<3, 1>()),
        ori(homog_transform.topLeftCorner<3, 3>()) {
    ori.normalize();
}

SE3::SE3(const Eigen::Affine3d& transform) :
        pos(transform.translation()), ori(transform.rotation()) {
    ori.normalize();
}

Eigen::Matrix4d
SE3::as_homogeneous_transformation() const {
    Eigen::Matrix4d res(Eigen::Matrix4d::Zero());
    res.topLeftCorner<3, 3>()  = ori.toRotationMatrix();
    res.topRightCorner<3, 1>() = pos;
    res(3, 3)                  = 1.0;
    return res;
}

Eigen::Affine3d
SE3::as_affine_transform() const {
    Eigen::Affine3d transf = Eigen::Affine3d::Identity();
    transf.translate(pos).rotate(ori);
    return transf;
}

Vec6_t
rm::logarithmic_map(const SE3& p, const SE3& x) {
    Vec6_t res;
    res.head<3>() = logarithmic_map(p.pos, x.pos);
    res.tail<3>() = logarithmic_map(p.ori, x.ori);
    return res;
}

SE3
rm::exponential_map(const SE3& p, const Vec6_t& v) {
    SE3          res;
    const Vec3_t pos_part = v.head<3>();
    const Vec3_t ori_part = v.tail<3>();
    res.pos               = exponential_map(p.pos, pos_part);
    res.ori               = exponential_map(p.ori, ori_part);
    return res;
}

bool
SE3::operator==(const SE3& other) const {
    return (this->pos == other.pos) && (this->ori == other.ori);
}
