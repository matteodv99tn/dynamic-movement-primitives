#ifndef DMP_SE3_MANIFOLD_HPP__
#define DMP_SE3_MANIFOLD_HPP__

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dmplib/manifolds/riemann_manifolds.hpp"
#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"

namespace dmp {

struct SE3 {
    Eigen::Vector3d    p;
    Eigen::Quaterniond q;

    SE3() {
        p = Eigen::Vector3d::Zero();
        q = Eigen::Quaterniond::Identity();
    }

    SE3(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation) {
        p = position;
        q = orientation;
    }

    SE3(const Eigen::Vector3d&& position, const Eigen::Quaterniond&& orientation) {
        p = position;
        q = orientation;
    }
};

class SE3Manifold : public RiemannManifold<SE3Manifold, SE3, 6> {
public:
    // By default, initialise all quaternions to the identity quaternion, i.e.
    // nu = 1, u = [0 0 0]
    static inline SE3
    construct_domain_impl() {
        return {R3Manifold::construct_domain_impl(), S3Manifold::construct_domain_impl()
        };
    }

    Tangent_t logarithmic_map(Eigen::Quaterniond q) const;

    Domain_t exponential_map(Tangent_t v) const;

protected:
    Tangent_t logarithmic_map_impl(const SE3& p, const SE3& qx) const;

    SE3 exponential_map_impl(const SE3& application_point, const Tangent_t& v) const;

private:
    R3Manifold _r3_manifold;  // sub-manifold for position computations
    S3Manifold _s3_manifold;  // sub-manifold for orientations computations
};

}  // namespace dmp

#endif  // DMP_SE3_MANIFOLD_HPP__
