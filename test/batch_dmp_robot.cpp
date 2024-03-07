#include "common/defines.hpp"
#include "common/test_functions.hpp"

int main() {
    Eigen::MatrixXd data = dmp::test::load_file("robot.csv");
    dmp::test::batch_learning_test(data, 48.0, 25, true, 1, dmp::test::LearningMethod::BATCH);

    return 0;
}