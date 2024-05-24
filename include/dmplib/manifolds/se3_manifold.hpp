#ifndef DMPLIB_SE3_MANIFOLD_HPP
#define DMPLIB_SE3_MANIFOLD_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>

#include "dmplib/manifolds/riemann_manifolds.hpp"
#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/manifolds/s3_manifold.hpp"

namespace dmp {

class SE3Manifold
        : public RiemannManifold<SE3Manifold, SE3> {  
                                                    
public:
    // By default, initialise all quaternions to the identity quaternion, i.e.
    // nu = 1, u = [0 0 0]
    [[nodiscard]] inline SE3
    construct_domain_impl() const {
        return {_r3_manifold.construct_domain_impl(),
                _s3_manifold.construct_domain_impl()};
    }

    [[nodiscard]] Tangent_t logarithmic_map(Eigen::Quaterniond q) const;

    [[nodiscard]] Domain_t exponential_map(Tangent_t v) const;

protected:
    [[nodiscard]] Tangent_t logarithmic_map_impl(const SE3& p, const SE3& qx) const;

    [[nodiscard]] SE3 exponential_map_impl(
            const SE3& application_point, const Tangent_t& v
    ) const;

private:
    R3Manifold _r3_manifold;  // sub-manifold for position computations
    S3Manifold _s3_manifold;  // sub-manifold for orientations computations
};

}  // namespace dmp

#endif  // DMPLIB_SE3_MANIFOLD_HPP
