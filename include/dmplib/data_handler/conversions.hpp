#ifndef DMPLIB_DATA_CONVERSIONS_HPP
#define DMPLIB_DATA_CONVERSIONS_HPP

#include <fmt/printf.h>
#include <string>
#include <cstdio>

#include "dmplib/data_handler/range_serialise.hpp"
#include "range/v3/range/concepts.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/intersperse.hpp"
#include "range/v3/view/transform.hpp"
#include "range/v3/view/join.hpp"

namespace dmp::to {

namespace rs = ::ranges;
namespace rv = ::ranges::views;

template <typename T>
std::vector<double>
vector(const T& obj) {
    static_assert(
            rs::range<decltype(dmp::ranges::serialise(obj))>,
            "Call to dmp::ranges::serialise<T> does not provide a valid range"
    );
    return dmp::ranges::serialise(obj) | rs::to<std::vector<double>>;
}

template <typename T>
std::vector<std::vector<double>>
vector(const std::vector<T>& objs) {
    return objs | rv::transform([](const auto& obj) { return vector(obj); })
           | rs::to<std::vector<std::vector<double>>>;
}

template <typename T>
std::string
string(const T& obj) {
    return ranges::serialise(obj)
           | rv::transform([](const auto& elem) { return std::to_string(elem); })
           | rv::intersperse(", ") | rv::join | rs::to<std::string>;
}

template <typename T>
std::vector<std::string>
string(const std::vector<T>& objs) {
    return objs
           | rv::transform([](const auto& obj) -> std::string { return string(obj); })
           | rs::to_vector;
}


}  // namespace dmp::to


#endif  // DMPLIB_DATA_CONVERSIONS_HPP
