#ifndef DMPLIB_RANGE_CONVERSIONS_HPP
#define DMPLIB_RANGE_CONVERSIONS_HPP

#include <concepts>
#include <Eigen/Geometry>
#include <string>
#include <tuple>
#include <vector>

#include "dmplib/data_handler/concepts.hpp"
#include "dmplib/manifolds/se3_manifold.hpp"
#include "range/v3/iterator/concepts.hpp"
#include "range/v3/range/concepts.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/join.hpp"
#include "range/v3/view/concat.hpp"
#include "range/v3/view/transform.hpp"

namespace dmp::ranges {

namespace rs = ::ranges;
namespace rv = ::ranges::views;

using StrVec_t = std::vector<std::string>;

namespace internal {
    template <::ranges::forward_range Iterable>
    [[nodiscard]] inline StrVec_t
    serialise_iterable(const Iterable& it) {
        return it | rv::transform([](const auto e) -> std::string {
                   return std::to_string(e);
               })
               | rs::to<StrVec_t>;
    }
}  // namespace internal

// Function that needs to be implemented
// If no specific definition is reported here, an automatic serialisation is attempted.
template <typename T>
[[nodiscard]] inline StrVec_t serialise(const T& obj);

// template <>
// [[nodiscard]] inline StrVec_t
// serialise<double>(const double& obj) {
//     return {std::to_string(obj)};
// }

template <concepts::stringable T>
[[nodiscard]] inline StrVec_t
serialise(const T& obj) {
    return {std::to_string(obj)};
}

template <::ranges::forward_iterator T>
[[nodiscard]] inline StrVec_t
serialise(const T& obj) {
    return internal::serialise_iterable(obj);
}

template <concepts::eigen_vector T>
[[nodiscard]] inline StrVec_t
serialise(const T& obj) {
    return internal::serialise_iterable(obj);
}

template <>
inline StrVec_t
serialise<Eigen::Quaterniond>(const Eigen::Quaterniond& obj) {
    return internal::serialise_iterable(obj.coeffs());
}

template <>
inline StrVec_t
serialise<riemannmanifold::SE3>(const riemannmanifold::SE3& obj){
    const auto pos_ser = serialise(obj.pos);
    const auto ori_ser = serialise(obj.ori);
    return rv::concat(pos_ser, ori_ser) | rs::to<StrVec_t>;
}


namespace internal {

    template <typename Tuple, std::size_t... Idxs>
    [[nodiscard]] inline StrVec_t
    tpl_unpack(const Tuple& tpl, std::index_sequence<Idxs...> /* unused */) {
        std::vector<StrVec_t> elems;
        (elems.push_back(serialise(std::get<Idxs>(tpl))), ...);
        return rv::join(elems) | rs::to<StrVec_t>;
    }

}  // namespace internal

// Tuple serialisation
template <typename... Args>
[[nodiscard]] inline StrVec_t
serialise(const std::tuple<Args...>& obj) {
    using Tpl_t = std::tuple<Args...>;
    return internal::tpl_unpack(
            obj, std::make_index_sequence<std::tuple_size_v<Tpl_t>>()
    );
}

// Vector serialisation
template <typename T>
inline std::vector<StrVec_t>
serialise(const std::vector<T>& obj_vec) {
    return obj_vec | rv::transform([](const auto& obj) { return serialise(obj); })
           | rs::to_vector;
}

}  // namespace dmp::ranges

#endif  // DMPLIB_RANGE_CONVERSIONS_HPP
