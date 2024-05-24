#ifndef DMPLIB_S3_MANIFOLD_HPP
#define DMPLIB_S3_MANIFOLD_HPP

#include <Eigen/Geometry>

#include "dmplib/manifolds/riemann_manifolds.hpp"

namespace dmp {

// Space of a quaternion
class S3Manifold : public RiemannManifold<S3Manifold, Eigen::Quaterniond> {
public:
    // By default, initialise all quaternions to the identity quaternion, i.e.
    // nu = 1, u = [0 0 0]
    [[nodiscard]] inline Eigen::Quaterniond
    construct_domain_impl() const { // NOLINT: don't want static members
        return Eigen::Quaterniond::Identity();
    }

    [[nodiscard]] Tangent_t logarithmic_map_single(Eigen::Quaterniond q) const;

    [[nodiscard]] Domain_t exponential_map_single(const Tangent_t& v) const;

    [[nodiscard]] Tangent_t logarithmic_map_impl(
            const Eigen::Quaterniond& qp, const Eigen::Quaterniond& qx
    ) const;

    [[nodiscard]] Domain_t exponential_map_impl(
            const Eigen::Quaterniond& application_point, const Tangent_t& v
    ) const;
};

}  // namespace dmp

#endif  // DMPLIB_S3_MANIFOLD_HPP
