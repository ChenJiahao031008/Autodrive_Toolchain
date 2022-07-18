#include "problem.hpp"

#ifndef LOG_CORE_DUMP_CAPTURE
#define BACKWARD_HAS_DW 1
#endif

int main(int argc, char** argv)
{
    common::Logger logger(argc, argv);

    Eigen::Vector2d x(0, 0);
    GradientDescent gd;
    gd.SetEpsilon(1e-3);
    gd.SetC(0.1);
    gd.SetTau(1.0);
    gd.SetVerbose(true);

    Rosenbrock2dExample rosenbrock(x);
    auto res = gd.Solve(rosenbrock);
    AINFO << "result = [" << res.transpose() << "]";
    AINFO << "f = " << rosenbrock.ComputeFunction(res);

    return 0;
}
