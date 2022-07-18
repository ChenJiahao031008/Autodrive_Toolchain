#pragma once

#include <ostream>
#include <fstream>
#include "../common/logger.hpp"
#include "cost_fuction.hpp"

namespace math_utils
{

class GradientDescent
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    double tau, init_tau;
    double c;
    double epsilon;
    int max_iters;
    bool verbose;

public:
    GradientDescent() : init_tau(tau), c(0.5), epsilon(1e-6), max_iters(100000), verbose(true){};

    GradientDescent(double tau_, double c_, double epsilon_, int max_iters_, bool verbose_)
        : init_tau(tau_), c(c_), epsilon(epsilon_),
          max_iters(max_iters_), verbose(verbose_) {};

    ~GradientDescent(){};

    inline void SetTau(double tau_) { init_tau = tau_; };
    inline void SetC(double c_) { c = c_; };
    inline void SetEpsilon(double epsilon_) { epsilon = epsilon_; };
    inline void SetMaxIters(int max_iters_) { max_iters = max_iters_; };
    inline void SetVerbose(bool verbose_) { verbose = verbose_; };

    // 智能指针不支持抽象类(?)
    template <typename _Scalar, int _Rows>
    Eigen::Matrix<_Scalar, _Rows, 1> Solve(CostFunction<_Scalar, _Rows> &cost_function)
    {
        auto x = cost_function.GetInitParam();
        if (verbose)
            AINFO << "Iter Count 0: Init vector is [" << x.transpose() << "]";

        double delta = cost_function.ComputeJacobian(x).norm();
        int iter_count = 0;
        while (delta >= epsilon && iter_count < max_iters)
        {
            tau = init_tau;
            while (ArmijoCondition<_Scalar, _Rows>(x, cost_function))
            {
                tau = tau * 0.5;
            }
            auto cur_d = cost_function.ComputeJacobian(x);
            x = x - tau * cur_d;
            delta = cur_d.norm();
            iter_count++;
            if (verbose)
            {
                AINFO << "Iter-->[" << iter_count << "/" << max_iters << "] delta: " << delta;
                AINFO << "Current Param: " << x.transpose();
            }
        }
        return x;
    };

    template <typename _Scalar, int _Rows>
    Eigen::Matrix<_Scalar, _Rows, 1> Solve(CostFunction<_Scalar, _Rows> &cost_function, std::ofstream& output)
    {
        auto x = cost_function.GetInitParam();
        if (verbose)
            AINFO << "Iter Count 0: Init vector is [" << x.transpose() << "]";
        output << std::fixed << std::setprecision(6);

        double delta = cost_function.ComputeJacobian(x).norm();
        int iter_count = 0;
        while (delta >= epsilon && iter_count < max_iters)
        {
            tau = init_tau;
            while (ArmijoCondition<_Scalar, _Rows>(x, cost_function))
            {
                tau = tau * 0.5;
            }
            auto cur_d = cost_function.ComputeJacobian(x);
            x = x - tau * cur_d;
            delta = cur_d.norm();
            iter_count++;
            double cur_f = cost_function.ComputeFunction(x);
            output << iter_count << " " << x.transpose() << " " << cur_f << std::endl;

            if (verbose)
            {
                AINFO << "Iter-->[" << iter_count << "/" << max_iters << "] delta: " << delta;
                AINFO << "Current Param: " << x.transpose();
            }

        }
        return x;
    };

private:
    template <typename _Scalar, int _Rows>
    bool ArmijoCondition(Eigen::Matrix<_Scalar, _Rows, 1> &x, CostFunction<_Scalar, _Rows> &cost_function)
    {
        Eigen::Matrix<_Scalar, _Rows, 1> d = cost_function.ComputeJacobian(x);
        double x_k0 = cost_function.ComputeFunction(x);
        double x_k1 = cost_function.ComputeFunction(x - d * tau);
        double left = x_k1 - x_k0;
        double right = -(c * d.transpose() * d * tau)[0];
        if (left >= right)
            return true;
        else
            return false;
    };
};


}
