#ifndef DMPLIB_DATA_CONVERSIONS_HPP
#define DMPLIB_DATA_CONVERSIONS_HPP

#include <cstdio>
#include <fmt/printf.h>
#include <fstream>
#include <string>

#include "dmplib/data_handler/range_deserialise.hpp"
#include "dmplib/data_handler/range_serialise.hpp"
#include "range/v3/range/concepts.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/intersperse.hpp"
#include "range/v3/view/istream.hpp"
#include "range/v3/view/join.hpp"
#include "range/v3/view/split.hpp"
#include "range/v3/view/transform.hpp"

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

namespace internal {
    void close_file(std::FILE* f);
}

template <typename T>
void
file(const std::string& file_name, const std::vector<T>& objs) {
    using FilePtr_t = std::unique_ptr<std::FILE, decltype(&internal::close_file)>;
    const FilePtr_t output_file(
            std::fopen(file_name.c_str(), "w"), &internal::close_file
    );
    if (!output_file) return;

    const std::vector<std::string> lines = string(objs);
    for (const auto& l : lines) fmt::print(output_file.get(), "{}", l);
}

}  // namespace dmp::to

namespace dmp::from {

namespace rs = ::ranges;
namespace rv = ::ranges::views;

namespace internal {
    std::vector<double> string_to_vec(const std::string& str);
}

template <typename T>
T
vector(const std::vector<double>& data) {
    return ranges::deserialise<T>(data);
}

template <typename T>
std::vector<T>
vector(const std::vector<std::vector<double>>& data) {
    return data | rv::transform([](const auto& row) -> T {
               return ::dmp::to::vector<T>(row);
           })
           | rs::to_vector;
}

template <typename T>
T
string(const std::string& content) {
    const std::vector<double> data = internal::string_to_vec(content);
    return ::dmp::from::vector<T>(data);
}

template <typename T>
std::vector<T>
string(const std::vector<std::string>& content) {
    return content | rv::transform([](const std::string& c) -> T { string<T>(c); })
           | rs::to_vector;
}

template <typename T>
std::vector<T>
file(const std::string& file_name) {
    std::ifstream  file_stream(file_name);
    std::string    line;
    std::vector<T> res;

    while (std::getline(file_stream, line)) {
        res.push_back(std::move(string<T>(line)));
    }
    return res;
}


}  // namespace dmp::from


#endif  // DMPLIB_DATA_CONVERSIONS_HPP
