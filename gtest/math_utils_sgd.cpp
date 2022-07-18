#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "../math_utils/cost_fuction.hpp"
#include "../math_utils/steepest_gradient_descent.hpp"
#include "../examples/SGD/problem.hpp"

TEST(TestUFreeOpt, TestSGDRosenbrock)
{
    Eigen::VectorXd x(5);
    x.setZero();
    Rosenbrock<5> rosenbrock(x);
    GradientDescent gd;
    gd.SetEpsilon(1e-6);
    gd.SetC(0.5);
    gd.SetTau(1.0);
    gd.SetMaxIters(10000);
    gd.SetVerbose(false); // 过程不打印
    auto res = gd.Solve(rosenbrock);
    Eigen::Matrix<double, 5, 1> true_result;
    true_result.setOnes();
    EXPECT_NEAR((res - true_result).norm(), 0, 1e-3);
}

TEST(TestUFreeOpt, TestSGDRosenbrock2d)
{
    Eigen::Vector2d x(0.0, 0.0);
    Rosenbrock2dExample rosenbrock(x);
    GradientDescent gd;
    gd.SetEpsilon(1e-6);
    gd.SetC(0.5);
    gd.SetTau(1.0);
    gd.SetMaxIters(10000);
    gd.SetVerbose(false); // 过程不打印
    auto res = gd.Solve(rosenbrock);
    Eigen::Vector2d true_result = {1.0, 1.0};
    EXPECT_NEAR((res - true_result).norm(), 0, 1e-3);
}

TEST(TestUFreeOpt, TestSGDPolynomial)
{
    Eigen::Vector2d x(0.0, 0.0);
    Example polynomial(x);
    GradientDescent gd;
    gd.SetEpsilon(1e-6);
    gd.SetC(0.5);
    gd.SetTau(1.0);
    gd.SetMaxIters(100);
    gd.SetVerbose(false); // 过程不打印
    auto res = gd.Solve(polynomial);
    Eigen::Vector2d true_result = {1.0, 1.0};
    EXPECT_NEAR((res - true_result).norm(), 0, 1e-3);
}
