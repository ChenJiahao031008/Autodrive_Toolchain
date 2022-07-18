#include <Eigen/Core>
#include <Eigen/Dense>
#include "../../common/logger.hpp"
#include "../../math_utils/cost_fuction.hpp"
#include "../../math_utils/steepest_gradient_descent.hpp"

using namespace math_utils;

/**
 * @brief :
 *      待优化求解的Rosenbrock函数, 继承自CostFunction基类, 来自wiki公式4
 * @param :
 *      double* param: 初值;
 */
template <int _Rows>
class Rosenbrock : public CostFunction<double, _Rows>
{
public:
    Rosenbrock(const Eigen::Matrix<double, _Rows, 1> &param) : CostFunction<double, _Rows>(param){};

    ~Rosenbrock() = default;

    double ComputeFunction(const Eigen::Matrix<double, _Rows, 1> &x) override
    {
        double result = 0;
        for (size_t i = 0; i < x.rows() - 1; ++i)
        {
            double part_1 = x[i + 1] - x[i] * x[i];
            double part_2 = 1 - x[i];
            result += 100.0 * part_1 * part_1 + part_2 * part_2;
        }
        return result;
    }

    Eigen::Matrix<double, _Rows, 1> ComputeJacobian(const Eigen::Matrix<double, _Rows, 1> &x) override
    {
        Eigen::Matrix<double, _Rows, 1> jacobian(x.rows());
        for (size_t i = 0; i < x.rows(); ++i)
        {
            if (i == 0)
            {
                jacobian(i, 0) = -400 * x[i] * (x[i + 1] - x[i] * x[i]) - 2 * (1 - x[i]);
            }
            else if (i == x.rows() - 1)
            {
                jacobian(i, 0) = 200 * (x[i] - x[i - 1] * x[i - 1]);
            }
            else
            {
                jacobian(i, 0) = -400 * x[i] * (x[i + 1] - x[i] * x[i]) - 2 * (1 - x[i]) + 200 * (x[i] - x[i - 1] * x[i - 1]);
            }
        }
        return jacobian;
    }
};

/**
 * @brief :
 *      待优化求解的Rosenbrock函数, 继承自CostFunction基类, 来自wiki公式3
 * @param :
 *      double* param: 初值;
 */
template <int _Rows>
class Rosenbrock2 : public CostFunction<double, _Rows>
{
public:
    Rosenbrock2(const Eigen::Matrix<double, _Rows, 1> &param) : CostFunction<double, _Rows>(param){};

    ~Rosenbrock2() = default;

    double ComputeFunction(const Eigen::Matrix<double, _Rows, 1> &x) override
    {
        double result = 0;
        for (size_t i = 0; i < x.rows() / 2; ++i)
        {
            double part_1 = x[2 * i] * x[2 * i] - x[2 * i + 1];
            double part_2 = x[2 * i] - 1;
            result += 100.0 * part_1 * part_1 + part_2 * part_2;
        }
        return result;
    }

    Eigen::Matrix<double, _Rows, 1> ComputeJacobian(const Eigen::Matrix<double, _Rows, 1> &x) override
    {
        Eigen::Matrix<double, _Rows, 1> jacobian(x.rows());
        jacobian.setZero();
        for (size_t i = 0; i < x.rows() / 2; i++)
        {
            jacobian(2 * i, 0) = 400 * x[2 * i] * (x[2 * i] * x[2 * i] - x[2 * i + 1]) + 2 * (x[2 * i] - 1);
            jacobian(2 * i + 1, 0) = -200 * (x[2 * i] * x[2 * i] - x[2 * i + 1]);
        }
        return jacobian;
    }
};

/**
 * @brief :
 *      待优化求解的多项式函数, 继承自CostFunction基类
 * @param :
 *      double* param: 初值;
 */
class Example : public CostFunction<double, 2>
{

public:
    Example(Eigen::Vector2d &param) : CostFunction<double, 2>(param){};

    ~Example() = default;

    double ComputeFunction(const Eigen::Vector2d &x) override
    {
        double result = 0;
        result = x[0] * x[0] + 2 * x[1] * x[1] - 2 * x[0] * x[1] - 2 * x[1];
        return result;
    }

    Eigen::Vector2d ComputeJacobian(const Eigen::Vector2d &x) override
    {
        Eigen::Vector2d jacobian(x.rows());
        jacobian(0, 0) = 2 * x[0] - 2 * x[1];
        jacobian(1, 0) = 4 * x[1] - 2 * x[0] - 2;
        return jacobian;
    }
};

/**
 * @brief :
 *      待优化求解的Rosenbrock函数, 继承自CostFunction基类，仅有两维便于可视化
 * @param :
 *      double* param: 初值;
 */
class Rosenbrock2dExample : public CostFunction<double, 2>
{
public:
    Rosenbrock2dExample(Eigen::Vector2d &param) : CostFunction<double, 2>(param){};

    ~Rosenbrock2dExample()=default;

    double ComputeFunction(const Eigen::Vector2d &x) override
    {
        double result = 0;
        double part_1 = x[0] * x[0] - x[1];
        double part_2 = x[0] - 1;
        result = 100.0 * part_1 * part_1 + part_2 * part_2;
        return result;
    }

    Eigen::Vector2d ComputeJacobian(const Eigen::Vector2d &x) override
    {
        Eigen::Vector2d jacobian(x.rows());
        jacobian(0, 0) = -2 * (1 - x[0]) - 400 * x[0] * ( x[1] - x[0] * x[0] );
        jacobian(1, 0) = 200 * (x[1] - x[0] * x[0]);
        return jacobian;
    }
};
