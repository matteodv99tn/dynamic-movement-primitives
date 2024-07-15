#ifndef DMPLIB_SE3_MANIFOLD_HPP
#define DMPLIB_SE3_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmplib/manifolds/riemann_manifold.hpp"

namespace dmp::riemannmanifold {

struct SE3 {  // NOLINT: naming convention
    Vec3_t       pos;
    Quaternion_t ori;

    SE3(Vec3_t       position    = Vec3_t::Zero(),
        Quaternion_t orientation = Quaternion_t::Identity());

    bool operator==(const SE3& other) const;
};

template <>
struct tangent_space_dimension<SE3> {
    static constexpr int value = 6;
};

template <>
struct constants<SE3> {
    static Mat6_t
    log_coefficient() {
        Mat6_t k = Mat6_t::Identity();
        k(3, 3)  = 2.0;
        k(4, 4)  = 2.0;
        k(5, 5)  = 2.0;
        return k;
    }

    static Mat6_t
    exp_coefficient() {
        Mat6_t k = Mat6_t::Identity();
        k(3, 3)  = 0.5;
        k(4, 4)  = 0.5;
        k(5, 5)  = 0.5;
        return k;
    }
};

Vec6_t logarithmic_map(const SE3& q1, const SE3& q2);
SE3    exponential_map(const SE3& q, const Vec6_t& v);

}  // namespace dmp::riemannmanifold

#endif  // DMPLIB_SE3_MANIFOLD_HPP
