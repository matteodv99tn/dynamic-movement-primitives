// #include "dmplib/data_handler/conversions.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include "range/v3/range/conversion.hpp"
#include "range/v3/view/split.hpp"
#include "range/v3/view/transform.hpp"

#include "dmplib/data_handler/conversions.hpp"

void
dmp::to::internal::close_file(std::FILE* f) {
    std::fclose(f);  // NOLINT
}

std::vector<std::string>
dmp::from::internal::string_to_vec(const std::string& str) {
    return str | rv::split(',')
           | rs::to<std::vector<std::string>>;
}
