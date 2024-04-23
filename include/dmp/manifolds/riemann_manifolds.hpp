#ifndef DMP_RIEMANN_MANIFOLDS_HPP__
#define DMP_RIEMANN_MANIFOLDS_HPP__

#include <Eigen/Dense>

namespace dmp {

    template<typename T>
    struct has_logarithmic_map_impl {
    private:
        template<typename C>
        static constexpr auto test(int) -> decltype(&C::logarithmic_map_impl, std::true_type{});
        template<typename C>
        static constexpr std::false_type test(...);

    public:
        static constexpr bool value = decltype(test<T>(0))::value;
    };


    template<typename T>
    struct has_exponential_map_impl {
    private:
        template<typename C>
        static constexpr auto test(int) -> decltype(&C::exponential_map_impl, std::true_type{});
        template<typename C>
        static constexpr std::false_type test(...);

    public:
        static constexpr bool value = decltype(test<T>(0))::value;
    };
    

    template<typename T>
    struct has_domain_constructor_impl {
    private:
        template<typename C>
        static constexpr auto test(int) -> decltype(&C::construct_domain_impl, std::true_type{});
        template<typename C>
        static constexpr std::false_type test(...);

    public:
        static constexpr bool value = decltype(test<T>(0))::value;
    };


    template <typename Derived, typename Domain, int SUBSPACE_DIM>
    class RiemannianManifold {
    public:
        using Domain_t = Domain;
        using Tangent_t = Eigen::Matrix<double, SUBSPACE_DIM, 1>;
        static constexpr int subspace_dim = SUBSPACE_DIM;


        Tangent_t construct_tangent() const {
            return Tangent_t::Zero();
        }

        Domain_t construct_domain() const {
            return static_cast<const Derived* const>(this)->construct_domain_impl();
        }

        /**
         * @brief Logarithmic map
         * 
         * Computes the logarithmic map of the manifold on the tangent space at the point x
         * of the point y.
         */
        Tangent_t logarithmic_map(const Domain_t& x, const Domain_t& y) const {
            return static_cast<const Derived* const>(this)->logarithmic_map_impl(x, y);
        }


        /**
         * @brief Exponential map
         * 
         * Computes the exponential mapping of the manifold at the point x of the tangent
         * vector y.
         */
        Domain_t exponential_map(const Domain_t& x, const Tangent_t& y) const {
            return static_cast<const Derived* const>(this)->exponential_map_impl(x, y);
        }
        
        RiemannianManifold() {
            
            // Check that the derived class implements the domain constructor
            static_assert(has_domain_constructor_impl<Derived>::value, "Derived class must implement Domain_t Derived::construct_domain_impl() const");

            // Check that the derived class implements the logarithmic map
            static_assert(has_logarithmic_map_impl<Derived>::value, "Derived class must implement Tangent_t Derived::logarithmic_map_impl(const Domain_t&, const Domain_t&) const");
            static_assert(std::is_invocable_r_v<Tangent_t, decltype(&Derived::logarithmic_map_impl), const Derived*, const Domain_t&, const Domain_t&>,
                          "Derived class must implement Tangent_t Derived::logarithmic_map_impl(const Domain_t&, const Domain_t&) const");
            
            // Check that the derived class implements the exponential map
            static_assert(has_exponential_map_impl<Derived>::value, "Derived class must implement Domain_t Derived::exponential_map_impl(const Domain_t&, const Tangent_t&) const");
            static_assert(std::is_invocable_r_v<Domain_t, decltype(&Derived::exponential_map_impl), const Derived*, const Domain_t&, const Tangent_t&>,
                          "Derived class must implement Domain_t Derived::exponential_map_impl(const Domain_t&, const Tangent_t&) const");
        }
    };
}

#endif // DMP_RIEMANN_MANIFOLDS_HPP__