#ifndef DMPLIB_RANGE_DESERIALISE_HPP
#define DMPLIB_RANGE_DESERIALISE_HPP

#include <Eigen/Geometry>
#include <concepts>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include "dmplib/data_handler/range_type_data.hpp"
#include "dmplib/data_handler/traits.hpp"
#include "range/v3/algorithm/copy.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"

#ifdef NDEBUG
#include "range/v3/view/drop.hpp"
#include "range/v3/view/take.hpp"
#else
#include "range/v3/view/drop_exactly.hpp"
#include "range/v3/view/take_exactly.hpp"
#endif

#include "dmplib/data_handler/concepts.hpp"

namespace dmp::ranges {

namespace rs     = ::ranges;
namespace rv     = ::ranges::views;
namespace traits = ::dmp::type_traits;

namespace internal {
    template <int N, typename Source>
    inline void
    deserialise_as_double(const Source& data, double* output) {
        auto to_double = [](const auto& str) -> double { return std::stod(str); };
#ifdef NDEBUG
        rs::copy(data | rv::transform(to_double) | rv::take(N), output);
#else
        rs::copy(data | rv::transform(to_double) | rv::take_exactly(N), output);

#endif
    }
}  // namespace internal

// template <typename Dest, typename Source>
// inline Dest deserialise(const Source& data);

template <concepts::eigen_quaternion Dest, typename Source>
[[nodiscard]] inline Dest
deserialise(const Source& data) {
    Eigen::Quaterniond q;
    internal::deserialise_as_double<4>(data, q.coeffs().data());
    q.normalize();
    return q;
}

template <concepts::eigen_vector Dest, typename Source>
[[nodiscard]] inline Dest
deserialise(const Source& data) {
    static_assert(Dest::RowsAtCompileTime != Eigen::Dynamic);
    Dest vec;
    internal::deserialise_as_double<Dest::RowsAtCompileTime>(data, vec.data());
    return vec;
}

template <std::floating_point Dest, typename Source>
[[nodiscard]] inline Dest
deserialise(const Source& data) {
    return std::stod(data[0]);
}

template <std::integral Dest, typename Source>
[[nodiscard]] inline Dest
deserialise(const Source& data) {
    return std::stoi(data[0]);
}

template <typename Dest, std::size_t From, typename Source>
inline Dest
deserialise_from(const Source& data) {
#ifdef NDEBUG
    return deserialise<Dest>(data | rv::drop(From));
#else
    return deserialise<Dest>(data | rv::drop_exactly(From));
#endif
}

namespace internal {

    template <typename Tuple, std::size_t... Idxs>
    constexpr std::size_t
    accumulate_prior_type_size(std::index_sequence<Idxs...> /*unused*/) {
        return (serialised_dimension<std::tuple_element_t<Idxs, Tuple>>::value + ...
        + 0
        );
    }

    template <typename Tuple, std::size_t Id>
    constexpr std::size_t
    prior_types_size() {
        if (Id == 0) return 0;
        return accumulate_prior_type_size<Tuple>(std::make_index_sequence<Id>());
    }

    template <typename Tuple, std::size_t Id, typename Source>
    inline std::tuple_element_t<Id, Tuple>
    deserialise_tuple_element(const Source& data) {
        return deserialise_from<
                std::tuple_element_t<Id, Tuple>,
                prior_types_size<Tuple, Id>()>(data);
    }

    template <typename Tuple, typename Source, std::size_t... Idxs>
    inline Tuple
    deserialise_tuple(const Source& data, std::index_sequence<Idxs...> /*unused*/) {
        return std::make_tuple(deserialise_tuple_element<Tuple, Idxs>(data)...);
    }

}  // namespace internal

template <typename Dest, typename Source>
inline std::enable_if_t<traits::is_tuple<Dest>::value, Dest>
deserialise(const Source& data) {
    return internal::deserialise_tuple<Dest>(
            data, std::make_index_sequence<std::tuple_size_v<Dest>>()
    );
}

namespace internal {
    template <typename T, typename Source>
    std::vector<T>
    deserialise_vector(const std::vector<Source>& data) {
        return data
               | rv::transform([](const auto& d) -> T { return deserialise<T>(d); })
               | rs::to<std::vector<T>>;
    }
}  // namespace internal

template <typename Dest, typename Source>
std::enable_if_t<traits::is_std_vector<Dest>::value, Dest>
deserialise(const std::vector<Source>& data) {
    return internal::deserialise_vector<traits::is_std_vector<Dest>::type>(data);
}

}  // namespace dmp::ranges

#endif  // DMPLIB_RANGE_DESERIALISE_HPP
