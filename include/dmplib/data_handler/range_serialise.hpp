#ifndef DMPLIB_RANGE_CONVERSIONS_HPP
#define DMPLIB_RANGE_CONVERSIONS_HPP

#include <Eigen/Geometry>
#include <tuple>
#include <vector>

#include "range/v3/iterator/concepts.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/all.hpp"
#include "range/v3/view/concat.hpp"
#include "range/v3/view/ref.hpp"
#include "range/v3/view/single.hpp"
#include "range/v3/view/transform.hpp"

namespace dmp::ranges {

namespace rs = ::ranges;
namespace rv = ::ranges::views;

// Function that needs to be implemented
// If no specific definition is reported here, an automatic serialisation is attempted.
template <typename T>
inline auto serialise(const T& obj);

template <>
inline auto serialise<double>(const double& obj) {
    return rv::single(obj);
}

template <>
inline auto
serialise<Eigen::Quaterniond>(const Eigen::Quaterniond& obj) {
    return rv::ref(obj.coeffs());
}

template <typename T>
inline auto
serialise(const T& obj) {
    static_assert(
            !rs::forward_iterator<T>, "Deduced serialise(obj) is not a forward iterator"
    );
    return rv::all(obj);
}

namespace internal {

    template <typename Tuple, std::size_t... Idxs>
    inline auto
    tpl_unpack(const Tuple& tpl, std::index_sequence<Idxs...> /*unused*/) {
        return rv::concat(serialise(std::get<Idxs>(tpl))...);
    }

}  // namespace internal

// Tuple serialisation
template <typename... Args>
inline auto
serialise(const std::tuple<Args...>& obj) {
    using Tpl_t = std::tuple<Args...>;
    return internal::tpl_unpack(
            obj, std::make_index_sequence<std::tuple_size_v<Tpl_t>>()
    );
}

// Vector serialisation
template <typename T, typename U>
inline std::vector<U>
serialise(const std::vector<T>& obj_vec) {
    return obj_vec | rv::transform([](const auto& obj) { return serialise(obj); })
           | rs::to_vector;
}

}  // namespace dmp::ranges

#endif  // DMPLIB_RANGE_CONVERSIONS_HPP
