#include "common/defines.hpp"
#include "common/test_functions.hpp"

int main() {
    Eigen::MatrixXd data = dmp::test::load_file("saveriano.csv");
    dmp::test::batch_learning_test(data, 48.0, 25, false, 4, dmp::test::LearningMethod::BATCH);
    return 0;
}
