#include "common/defines.hpp"
#include "common/test_functions.hpp"

#include <iostream>
int main() {
    Eigen::MatrixXd data = dmp::test::load_file("synthetic.csv");
    dmp::test::batch_learning_test(data, 48.0, 50, true, 1);
    return 0;
}
