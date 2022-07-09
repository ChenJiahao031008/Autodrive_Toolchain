#include <gtest/gtest.h>

#include "../common/logger.hpp"
#include "../math_utils/curve_fitting.hpp"

TEST(TestMathUtils, TestFitPolynomial)
{
    std::vector<double> x = {1, 2, 3, 4, 5,  6,  7,  8,  9, 10};
    std::vector<double> y = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19};
    std::array<Eigen::Vector2d, 10> points;
    for (int i = 0; i < 10; ++i) {
        Eigen::Vector2d pt;
        pt << x[i], y[i];
        points.at(i) = pt;
    }
    double *ptr_error_square = nullptr;
    auto coeff = math_utils::FitPolynomial<2UL, 10UL>(points, ptr_error_square);
    EXPECT_NEAR(coeff.at(0), -1, 1e-6);
    EXPECT_NEAR(coeff.at(1), 2, 1e-6);
    EXPECT_NEAR(coeff.at(2), 0, 1e-6);
}
