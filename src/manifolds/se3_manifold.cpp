#include "dmplib/manifolds/se3_manifold.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>

#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"

using namespace dmp::riemannmanifold;

SE3::SE3() : pos(Vec3_t::Zero()), ori(Quaternion_t::Identity()) {
}

Vec6_t
logarithmic_map(const SE3& p, const SE3& x) {
    Vec6_t res;
    res.head<3>() = logarithmic_map(p.pos, x.pos);
    res.tail<3>() = logarithmic_map(p.ori, x.ori);
    return res;
}

SE3
exponential_map(const SE3& p, const Vec6_t& v) {
    SE3 res;
    Vec3_t pos_part = v.head<3>();
    Vec3_t ori_part = v.tail<3>();
    res.pos = exponential_map(p.pos, pos_part);
    res.ori = exponential_map(p.ori, ori_part);
    return res;
}
