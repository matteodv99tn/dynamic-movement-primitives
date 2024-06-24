#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
int main() { // NOLINT Bugprone-exception-escape
    plt::plot({1,3,2,4});
    plt::show();
}
