#include "dmplib/manifolds/se3_manifold.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>
#include <utility>

#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"

namespace rm = dmp::riemannmanifold;

using rm::SE3;
using rm::Vec3_t;
using rm::Vec6_t;

SE3::SE3(Vec3_t position, Quaternion_t orientation) :
        pos(std::move(position)), ori(std::move(orientation)){};

Vec6_t
rm::logarithmic_map(const SE3& p, const SE3& x) {
    Vec6_t res;
    res.head<3>() = logarithmic_map(p.pos, x.pos);
    res.tail<3>() = logarithmic_map(p.ori, x.ori);
    return res;
}

SE3
rm::exponential_map(const SE3& p, const Vec6_t& v) {
    SE3    res;
    const Vec3_t pos_part = v.head<3>();
    const Vec3_t ori_part = v.tail<3>();
    res.pos         = exponential_map(p.pos, pos_part);
    res.ori         = exponential_map(p.ori, ori_part);
    return res;
}

bool SE3::operator==(const SE3& other) const {
    return (this->pos == other.pos) && (this->ori == other.ori);
}
