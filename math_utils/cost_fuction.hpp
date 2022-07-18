#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include "../common/logger.hpp"

namespace math_utils
{

    template <typename _Scalar, int _Rows>
    class CostFunction
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        const int N;
        Eigen::Matrix<_Scalar, _Rows, 1> x;

    public:
        CostFunction(const Eigen::Matrix<_Scalar, _Rows, 1> &param) : N(param.rows()), x(param){};

        virtual ~CostFunction() = default;

        virtual double ComputeFunction(const Eigen::Matrix<_Scalar, _Rows, 1> &x) = 0;

        virtual Eigen::Matrix<_Scalar, _Rows, 1> ComputeJacobian(const Eigen::Matrix<_Scalar, _Rows, 1> &x) = 0;

        Eigen::Matrix<_Scalar, _Rows, 1> GetInitParam() { return x; };

        int GetParamSize() { return N; };
    };

} // namespace math_utils
