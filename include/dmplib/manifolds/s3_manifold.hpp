#ifndef DMP_S3_MANIFOLD_HPP__
#define DMP_S3_MANIFOLD_HPP__

#include <Eigen/Geometry>

#include "dmplib/manifolds/riemann_manifolds.hpp"

namespace dmp {

// Space of a quaternion
class S3Manifold : public RiemannManifold<S3Manifold, Eigen::Quaterniond, 3> {
public:
    // By default, initialise all quaternions to the identity quaternion, i.e.
    // nu = 1, u = [0 0 0]
    inline Eigen::Quaterniond
    construct_domain_impl() const {
        return Eigen::Quaterniond::Identity();
    }

    Tangent_t logarithmic_map_single(Eigen::Quaterniond q) const;

    Domain_t exponential_map_single(Tangent_t v) const;

    Tangent_t logarithmic_map_impl(
            const Eigen::Quaterniond& qp, const Eigen::Quaterniond& qx
    ) const;

    Domain_t exponential_map_impl(
            const Eigen::Quaterniond& application_point, const Tangent_t& v
    ) const;
};

}  // namespace dmp

#endif  // DMP_S3_MANIFOLD_HPP__
