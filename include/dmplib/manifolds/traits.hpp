#ifndef DMPLIB_MANIFOLDS_TRAITS_HPP
#define DMPLIB_MANIFOLDS_TRAITS_HPP

#include <type_traits>

namespace dmp {

template <typename T>
struct has_custom_constructor {
private:
    template <typename U>
    static auto test(int)
            -> decltype(std::declval<U>().construct_domain_impl(), std::true_type{});

    template <typename>
    static std::false_type test(...);

public:
    // Define the value alias that checks if the function has the given function
    static constexpr bool value = decltype(test<T>(0))::value;
};

template <typename T>
static constexpr bool has_custom_constructor_v = has_custom_constructor<T>::value;

}  // namespace dmp


#endif  // DMPLIB_MANIFOLDS_TRAITS_HPP
