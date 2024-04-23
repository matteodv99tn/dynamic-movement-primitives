#ifndef DMP_S3_MANIFOLD_HPP__
#define DMP_S3_MANIFOLD_HPP__

#include "dmp/manifolds/riemann_manifolds.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace dmp{

    // Space of a quaternion
    class S3Manifold : public RiemannianManifold<S3Manifold, Eigen::Quaterniond, 3> {
    public:

        Eigen::Quaterniond construct_domain_impl() const {
            return Eigen::Quaterniond::Identity();
        }

        Tangent_t logarithmic_map_impl(const Eigen::Quaterniond& application_point, const Eigen::Quaterniond& y) const {
            return 2 * log_map(y * application_point.conjugate());
        }

        Tangent_t log_map(Eigen::Quaterniond q) const {
            if (q.w() < 0) {
                q.coeffs() = -q.coeffs();
            }
            const Eigen::Vector3d u = q.vec();
            const double nu = q.w();

            if( u.norm() < 1e-6 ) 
                return Eigen::Vector3d::Zero();
            

            return std::acos(nu) * u.normalized();
        }
        
        Domain_t exponential_map_impl(const Eigen::Quaterniond& application_point, const Tangent_t& v) const {
            return Eigen::Quaterniond::Identity();
        }
    };

}

#endif // DMP_S3_MANIFOLD_HPP__