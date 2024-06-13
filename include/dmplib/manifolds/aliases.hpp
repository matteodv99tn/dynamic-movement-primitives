#ifndef DMPLIB_RIEMANN_MANIFOLD_ALIASES_HPP
#define DMPLIB_RIEMANN_MANIFOLD_ALIASES_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace dmp::riemannmanifold {

    template <int N>
    using Vec_t = Eigen::Matrix<double, N, 1>;

    using Vec2_t = Eigen::Vector2d;
    using Vec3_t = Eigen::Vector3d;
    using Vec6_t = Vec_t<6>;
    using Quaternion_t = Eigen::Quaterniond;


    


} // namespace dmp::riemannmanifold


#endif  // DMPLIB_RIEMANN_MANIFOLD_ALIASES_HPP
