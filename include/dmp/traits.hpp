#ifndef DMP_TYPE_TRAITS_HPP__
#define DMP_TYPE_TRAITS_HPP__

#include <type_traits>

namespace dmp {
    namespace traits {

        template <typename T>
        concept riemann_manifold = requires(T manifold) {
            // Check typename definitions
            typename T::Domain_t;
            typename T::Tangent_t;


            // Check if the logarithmic map is defined
            {
                manifold.logarithmic_map(
                        std::declval<typename T::Domain_t>(),
                        std::declval<typename T::Domain_t>()
                )
                } -> std::same_as<typename T::Tangent_t>;

            // Check if the exponential map is defined
            {
                manifold.exponential_map(
                        std::declval<typename T::Domain_t>(),
                        std::declval<typename T::Tangent_t>()
                )
                } -> std::same_as<typename T::Domain_t>;
        };
    }  // namespace traits
}  // namespace dmp

#endif  // DMP_TYPE_TRAITS_HPP__
