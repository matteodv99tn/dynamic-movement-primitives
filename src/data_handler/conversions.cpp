#include "dmplib/data_handler/conversions.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include "range/v3/range/conversion.hpp"
#include "range/v3/view/split.hpp"
#include "range/v3/view/transform.hpp"

void
dmp::to::internal::close_file(std::FILE* f) {
    std::fclose(f);  // NOLINT
}

std::vector<double>
dmp::from::internal::string_to_vec(const std::string& str) {
    return str | rv::split(',')
           | rv::transform([](const auto&& rng) -> double { return std::stod(rng | rs::to<std::string>); })
           | rs::to_vector;
}
