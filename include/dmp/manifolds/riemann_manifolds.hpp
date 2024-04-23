#ifndef DMP_RIEMANN_MANIFOLDS_HPP__
#define DMP_RIEMANN_MANIFOLDS_HPP__

#include <Eigen/Dense>
#include <type_traits>

#include "dmp/manifolds/traits.hpp"

namespace dmp {

/**
 * @brief Riemann Manifold base class for CRTP
 *
 * This class provides some common definition to easen the development of the DMP later
 * on. The CRTP pattern expects as first input the Derived manifold itself, the object
 * type used to encode the manifold, and the dimension of the tangent subspace.
 *
 * @tparam Derived Child manifold for CRTP
 * @tparam Domain Type of the domain obj. used for computation
 * @tparam SUBSPACE_DIM Dimension of the embedding space
 */
template <typename Derived, typename Domain, int SUBSPACE_DIM>
class RiemannManifold {
public:
    using Domain_t                    = Domain;
    using Tangent_t                   = Eigen::Matrix<double, SUBSPACE_DIM, 1>;
    static constexpr int subspace_dim = SUBSPACE_DIM;

    static inline Tangent_t
    construct_tangent() {
        return Tangent_t::Zero();
    }

    static inline Domain_t
    construct_domain() {
        static_assert(
                dmp::has_custom_constructor_v<Derived> ||
                        std::is_default_constructible<Domain_t>::value,
                "The Domain of the manifold is not default constructible, and the "
                "implementation did not provide a valid constructor"
        );

        if constexpr (dmp::has_custom_constructor_v<Derived>)
            return Derived::construct_domain_impl();

        if constexpr (std::is_default_constructible<Domain_t>::value) return Domain_t();
    }

    /**
     * @brief Logarithmic map operation
     *
     * Computes the tangent vector of the point x on the given manifold, with tangent
     * space computed on the point p.
     *
     * @param[input] p point w.r.t. which the tangent space is referred
     * @param[input] x point that's projected in the tangent space
     * @return the tangent vector of the operation
     */
    Tangent_t
    logarithmic_map(const Domain_t& p, const Domain_t& x) const {
        return static_cast<const Derived* const>(this)->logarithmic_map_impl(p, x);
    }

    /**
     * @brief Exponential map operation
     *
     * Returns the point in the domain that is obtained, starting from the point p on
     * the manifold, by integrating by a vector v on the tangent space computed in such
     * point. Optionally, the vector can be rescaled by a sampling period dt.
     *
     * @param[input] p point on the manifold w.r.t. which the tangent space is defined
     * and integration is performed
     * @param[input] v integration vector
     * @param[input] dt scaling term of the vector v, defaults to 1
     * @return the point after integration
     */
    Domain_t
    exponential_map(const Domain_t& p, const Tangent_t& v, const double& dt = 1) const {
        return static_cast<const Derived* const>(this)->exponential_map_impl(p, v * dt);
    }
};
}  // namespace dmp

#endif  // DMP_RIEMANN_MANIFOLDS_HPP__