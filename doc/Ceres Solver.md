# Ceres Solver

## CeresSlover之Helloworld实现

### 求解优化问题

Ceres 问题求解主要分成以下三部分:

+ 构建 `cost function`, 即代价函数, 也就是寻优的目标式(通过预测结果和测量值求误差的函数)。这个部分需要使用函子（`functor`）这一技巧来实现;

+ 通过上一步的代价函数构建待求解的优化问题;

+ 配置求解器参数并求解问题, 这个步骤就是设置方程怎么求解、求解过程是否输出等, 然后调用一下 Solve 方法。 

```c++
#include<iostream>
#include<ceres/ceres.h>
using namespace std;
//第一部分：构建代价函数
struct CostFunctor {
//模板函数, 泛化函数类型
template <typename T>
//重载符号 (), 仿函数; 传入待优化变量列表和承接残差的变量列表
bool operator()(const T* const x, T* residual) const {
//残差计算步骤//其中第一个const代表指针指向的是一个const类型的东西，
    //第二个const代表的是x是不可以发生变的
    //第三个const代表的是这个函数，不改变类的成员变量
residual[0] =T(0.5)*(T(10.0)-x[0])*(T(10.0)-x[0]) ; / / 1 / 2(10-x)^ 2$
return true;
}
};
//主函数
int main(int argc, char** argv)
{
// 寻优参数 x 的初始值, 为 5
double initial_ x=5.0;
double x= initial_ x;
// 第二部分：构建寻优问题
//实例化 Problem
ceres::Problem problem;
//代价函数跒值
//使用自动求导, 将之前的代价函数结构体传入, 第一个 1 是输出维度, 即残差的维度, 第二个 1 是输入维度, 即待寻优参数 x 的维度。
 ceres::CostFunction* cost_function = new AutoDiffCostFunction ⟨ CostFunctor, 1,1>( new CostFunctor);
//添加误差项, 1 、上一步实例化后的代价函数 2 、核函数 3 、待优化变量  
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor), nullptr, &x);
//第三部分： 配置并运行求解器
ceres::Solver::Options options;
//配置增量方程的解法, 此处为 Q R求解
options.linear_solver_type = ceres::DENSETQR;
//是否输出到 cout
options.minimizer_progress_to_stdout =true;
//优化信息
ceres::Solver::Summary summary;
//求解: 1、求解器 2 、实例化 problem 3 、优化器
Solve(options,&problem, &summary);
//输出优化的简要信息,迭代次数和每次的 cost
std::cout << summary.BriefReport()<< "\n";
    //最终结果
std::cout << "初始值 x: " << initial_x<<" 迭代到 ->" <<x<< " \n " ;
return 0 ;
}
```

### 分析
#### 第一部分：构建代价函数结构体
`CostFunction `结构体中, 对括号符号重载的函数中, 传入参数有两个, 一个是待优化的变量 $x$, 另一个是残差 residual, 也就是代价函数的输出。

```c++
//第一部分：构建代价函数，重载（）符号，仿函数的小技巧
struct CostFunctor {
   template <typename T>
   bool operator()(const T* const x, T* residual) const {
     residual[0] = T(10.0) - x[0];
     return true;
   }
};

```

#### 第二部分：通过代价函数构建待求解的优化问匙
这一步最主要的部分是残差块添加函数: `AddResidualBlock `的使用, 涉及到的三个参数分别 是

+ 自动求导代价函数;

+ 是否添加核函数;

+ 待优化变量。

其中自动求导函数 `AutoDiffCostFunction `可以单独进行, 它内部的三个参数分别为

+ 上述定义的代价函数结构体;
+ 残差维度;
+ 待优化变量的维度。

```c++
// 第二部分：构建寻优问题
//实例化 Problem
ceres::Problem problem;
//代价函数跒值
//使用自动求导, 将之前的代价函数结构体传入, 第一个 1 是输出维度, 即残差的维度, 第二个 1 是输入维度, 即待寻优参数 x 的维度。
 ceres::CostFunction* cost_function = new AutoDiffCostFunction ⟨ CostFunctor, 1,1>( new CostFunctor);
//添加误差项, 1 、上一步实例化后的代价函数 2 、核函数 3 、待优化变量  
  problem.AddResidualBlock(cost_function, nullptr, &x);
```



#### 第三部分：配置优化器执行优化
这一部分实现求解器实例化; 选择求解方式 (这里用 $Q R$ 分解 ; 是否输出运行信息; 优化器 实例化; 调用 Slove 函数进行问题求解；简要输出执行信息。
其中 `Slove `函数很重要, 它负责最后的问题求解, 涉及到的三个参数分别是

+  求解器实例化;
+ 优化问题实例化; 
+ 优化器实例化。

```c++
//第三部分： 配置并运行求解器
ceres::Solver::Options options;
//配置增量方程的解法, 此处为 Q R求解
options.linear_solver_type = ceres::DENSETQR;
//是否输出到 cout
options.minimizer_progress_to_stdout =true;
//优化信息
ceres::Solver::Summary summary;
//求解: 1、求解器 2 、实例化 problem 3 、优化器
Solve(options,&problem, &summary);
//输出优化的简要信息,迭代次数和每次的 cost
std::cout << summary.BriefReport()<< "\n";
    //最终结果
```

### 总结

学习完本部分后，依托着简单的函数，我们可以大致总结出ceres解决的问题的框架和步骤：

1.**建立`constFuntions`** 
2.**设置初值，构建出最小二乘问题（使用`Probolem`类）**
3.**构建`solver`，设置使用的方法如线性最小二乘等**
4.**输出程序结果** 

## ceres在powell法上的实现

接下来介绍一个较为复杂的问题，鲍威尔优化算法。有兴趣的可以查询其算法细节，本文主要是针对ceres的使用。 参数为 $\mathrm{x}=\left[\mathrm{x}_1, \mathrm{x}_2, \mathrm{x}_3, \mathrm{x}_4\right]$, 具体的函数为:
$$
\begin{gathered}
\mathrm{f}_1(\mathrm{x})=\mathrm{x} 1+10 \mathrm{x}_2 \\
\mathrm{f}_2(\mathrm{x})=\sqrt{5}\left(\mathrm{x}_3-\mathrm{x}_4\right) \\
\mathrm{f}_3(\mathrm{x})=\left(\mathrm{x}_2-2 \mathrm{x}_3\right)^2 \\
\mathrm{f}_4(\mathrm{x})=\sqrt{10}\left(\mathrm{x}_1-\mathrm{x}_4\right)^2 \\
\mathrm{~F}(\mathrm{x})=\left[\mathrm{f}_1(\mathrm{x}), \mathrm{f}_2(\mathrm{x}), \mathrm{f}_3(\mathrm{x}), \mathrm{f}_4(\mathrm{x})\right]
\end{gathered}
$$
$\mathrm{F}(\mathrm{x})$ 有四个参数，所有四个残差，找到一个 $\mathrm{x}$ 让 $\frac{1}{2}\|\mathrm{~F}(\mathrm{x})\|^2$ 的值最小。

### 构建costFunctior

对于单独的 $f_4(x)$ 来说:

```c++
struct F4 {
  template <typename T>
  //f_4(x)的相关参量为x_1和x_4
  bool operator()(const T* const x1, const T* const x4, T* residual) const {
  //按照f_4(x)构建残差公式
    residual[0] = T(sqrt(10.0)) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
    return true;
  }
};
```

完整的构建为：

```c++
struct F1 {
  template <typename T> bool operator()(const T* const x1,
                                        const T* const x2,
                                        T* residual) const {
    // f1 = x1 + 10 * x2;
    residual[0] = x1[0] + 10.0 * x2[0];
    return true;
  }
};

struct F2 {
  template <typename T> bool operator()(const T* const x3,
                                        const T* const x4,
                                        T* residual) const {
    // f2 = sqrt(5) (x3 - x4)
    residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
    return true;
  }
};

struct F3 {
  template <typename T> bool operator()(const T* const x2,
                                        const T* const x3,
                                        T* residual) const {
    // f3 = (x2 - 2 x3)^2
    residual[0] = (x2[0] - 2.0 * x3[0]) * (x2[0] - 2.0 * x3[0]);
    return true;
  }
};

struct F4 {
  template <typename T> bool operator()(const T* const x1,
                                        const T* const x4,
                                        T* residual) const {
    // f4 = sqrt(10) (x1 - x4)^2
    residual[0] = sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
    return true;
  }
};

```

### 构造最小二乘问题的框架

```c++
 Problem problem;
  // 还是用autoDiff的方式来求解导数，前面是costfunction的规模，后面是相关的参量，使用&来表明
  problem.AddResidualBlock(new AutoDiffCostFunction<F1, 1, 1, 1>(new F1),NULL,&x1, &x2);
  problem.AddResidualBlock(new AutoDiffCostFunction<F2, 1, 1, 1>(new F2),NULL,&x3, &x4);
  problem.AddResidualBlock(new AutoDiffCostFunction<F3, 1, 1, 1>(new F3), NULL,&x2, &x3);
  problem.AddResidualBlock(new AutoDiffCostFunction<F4, 1, 1, 1>(new F4),NULL,&x1, &x4);


```

### 构建Solver并输出结果

```c++
//Solver的配置类
Solver::Options options;
  LOG_IF(FATAL, !ceres::StringToMinimizerType(FLAGS_minimizer,
                                              &options.minimizer_type))
      << "Invalid minimizer: " << FLAGS_minimizer
      << ", valid options are: trust_region and line_search.";

  options.max_num_iterations = 100;//设置最大迭代次数
  options.linear_solver_type = ceres::DENSE_QR;//使用稠密QR方法解决
  options.minimizer_progress_to_stdout = true;//可以输出过程变量

  std::cout << "Initial x1 = " << x1
            << ", x2 = " << x2
            << ", x3 = " << x3
            << ", x4 = " << x4
            << "\n";

  // 运行solver!
  Solver::Summary summary;//用来放中间报告
  Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << "\n";
  std::cout << "Final x1 = " << x1
            << ", x2 = " << x2
            << ", x3 = " << x3
            << ", x4 = " << x4
            << "\n";

```

运行结果为：

```c++
Final x1 = 0.000292189, x2 = -2.92189e-05, x3 = 4.79511e-05, x4 = 4.79511e-05
```

## ceres在曲线拟合（Curve Fitting）中的实现

本部分为本笔记的重点，是我们第一次真正意义上的使用ceres解决非线性最小二乘问题，当然，结构是还是一致的，只需要稍加改动就 可以实现。
数据来源：
我们来拟合曲线:
$$
\mathrm{y}=\mathrm{e}^{\mathrm{mx}+\mathrm{c}}
$$

###  建立损失函数与残差

和前文一致，先建立损失函数，**此处和前面不同的是第一次引入了数据**，所以`residual`和前面有所区别，但这个位真正的非线性最小二乘的结构。

```c++
struct ExponentialResidual {
//定义数据的x和y的析构函数
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T>
  //两个待估参量m，c类型为指向T* 的指针
  bool operator()(const T* const m, const T* const c, T* residual) const {
   	//残差为一组数据中 r = y - f(x)
    residual[0] = T(y_) - exp(m[0] * T(x_) + c[0]);
    return true;
  }

 private:
  // 一组观测样本
  const double x_;
  const double y_;
};

```

### 构建最小二乘问题

```c++
//设置初值
double m = 0.0;
double c = 0.0;
//构建最小二乘问题
Problem problem;
for (int i = 0; i < kNumObservations; ++i) {
  CostFunction* cost_function =
  		//残差维度为1，m维度为1，c维度为1：这就是三个1的意思
       new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
       //data中偶数位存放x的样本值，奇数位存放y的样本值
           new ExponentialResidual(data[2 * i], data[2 * i + 1]));
           //残差的参量是m和c
  problem.AddResidualBlock(cost_function, NULL, &m, &c);
}
```

### 构建Solver并输出结果

```c++
  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
  std::cout << "Final   m: " << m << " c: " << c << "\n";


```

### 结果及分析

```c++
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

// Data generated using the following octave code.
//   randn('seed', 23497);
//   m = 0.3;
//   c = 0.1;
//   x=[0:0.075:5];
//   y = exp(m * x + c);
//   noise = randn(size(x)) * 0.2;
//   y_observed = y + noise;
//   data = [x', y_observed'];

const int kNumObservations = 67;
const double data[] = {
  0.000000e+00, 1.133898e+00,
  7.500000e-02, 1.334902e+00,
  1.500000e-01, 1.213546e+00,
  2.250000e-01, 1.252016e+00,
  3.000000e-01, 1.392265e+00,
  3.750000e-01, 1.314458e+00,
  4.500000e-01, 1.472541e+00,
  5.250000e-01, 1.536218e+00,
  6.000000e-01, 1.355679e+00,
  6.750000e-01, 1.463566e+00,
  7.500000e-01, 1.490201e+00,
  8.250000e-01, 1.658699e+00,
  9.000000e-01, 1.067574e+00,
  9.750000e-01, 1.464629e+00,
  1.050000e+00, 1.402653e+00,
  1.125000e+00, 1.713141e+00,
  1.200000e+00, 1.527021e+00,
  1.275000e+00, 1.702632e+00,
  1.350000e+00, 1.423899e+00,
  1.425000e+00, 1.543078e+00,
  1.500000e+00, 1.664015e+00,
  1.575000e+00, 1.732484e+00,
  1.650000e+00, 1.543296e+00,
  1.725000e+00, 1.959523e+00,
  1.800000e+00, 1.685132e+00,
  1.875000e+00, 1.951791e+00,
  1.950000e+00, 2.095346e+00,
  2.025000e+00, 2.361460e+00,
  2.100000e+00, 2.169119e+00,
  2.175000e+00, 2.061745e+00,
  2.250000e+00, 2.178641e+00,
  2.325000e+00, 2.104346e+00,
  2.400000e+00, 2.584470e+00,
  2.475000e+00, 1.914158e+00,
  2.550000e+00, 2.368375e+00,
  2.625000e+00, 2.686125e+00,
  2.700000e+00, 2.712395e+00,
  2.775000e+00, 2.499511e+00,
  2.850000e+00, 2.558897e+00,
  2.925000e+00, 2.309154e+00,
  3.000000e+00, 2.869503e+00,
  3.075000e+00, 3.116645e+00,
  3.150000e+00, 3.094907e+00,
  3.225000e+00, 2.471759e+00,
  3.300000e+00, 3.017131e+00,
  3.375000e+00, 3.232381e+00,
  3.450000e+00, 2.944596e+00,
  3.525000e+00, 3.385343e+00,
  3.600000e+00, 3.199826e+00,
  3.675000e+00, 3.423039e+00,
  3.750000e+00, 3.621552e+00,
  3.825000e+00, 3.559255e+00,
  3.900000e+00, 3.530713e+00,
  3.975000e+00, 3.561766e+00,
  4.050000e+00, 3.544574e+00,
  4.125000e+00, 3.867945e+00,
  4.200000e+00, 4.049776e+00,
  4.275000e+00, 3.885601e+00,
  4.350000e+00, 4.110505e+00,
  4.425000e+00, 4.345320e+00,
  4.500000e+00, 4.161241e+00,
  4.575000e+00, 4.363407e+00,
  4.650000e+00, 4.161576e+00,
  4.725000e+00, 4.619728e+00,
  4.800000e+00, 4.737410e+00,
  4.875000e+00, 4.727863e+00,
  4.950000e+00, 4.669206e+00,
};

struct ExponentialResidual {
  ExponentialResidual(double x, double y)
      : x_(x), y_(y) {}

  template <typename T> bool operator()(const T* const m,
                                        const T* const c,
                                        T* residual) const {
    residual[0] = y_ - exp(m[0] * x_ + c[0]);
    return true;
  }

 private:
  const double x_;
  const double y_;
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  double m = 0.0;
  double c = 0.0;

  Problem problem;
  for (int i = 0; i < kNumObservations; ++i) {
    problem.AddResidualBlock(
        new AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
            new ExponentialResidual(data[2 * i], data[2 * i + 1])),
        NULL,
        &m, &c);
  }

  Solver::Options options;
  options.max_num_iterations = 25;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
  std::cout << "Final   m: " << m << " c: " << c << "\n";
  return 0;


```

输出结果是：
```c++
Final   m: 0.291861 c: 0.131439
```

## 使用解析求导

使用解析求导构建残差函数时，一般会有两个可以继承的类：`ceres::CostFunction`以及`ceres::SizedCostFunction`,我们通过对比可以仔细发现其实`ceres::SizedCostFunction`也是继承了CostFunction的， **通过查看其构造函数可以发现我们刚才直接继承`CostFunction`构建`functor`的方法其实和`SizedCostFunction`是一样的。
只不过Ceres已经帮我们做好了，**以后我们只使用`SizedCostFunction`就好了，很方便。

在使用时我们需要传入，**残差的维度**以及**参数的维度**

```c++
  SizedCostFunction() {
    set_num_residuals(kNumResiduals);
    *mutable_parameter_block_sizes() = std::vector<int32_t>{Ns...};
  }

```

解析求导使用例程：
```c++
//-------------------------------------------
// 解析求导方式
//-------------------------------------------
class AnalyticCostFunction
        : public ceres::SizedCostFunction<1 /* number of residuals */,
                2 /* size of first parameter */> {
public:
    AnalyticCostFunction(double x, double y) : x_(x), y_(y) {}
    virtual ~AnalyticCostFunction() {}

    /**
     * @brief 重载Evaluate函数，完成jacobian和residuals的计算
     * @param parameters
     * @param residuals
     * @param jacobians
     * @return
     */
    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        double m = parameters[0][0]; // parameters[0]表示取出第一组参数
        double c = parameters[0][1];

        // 计算残差
        residuals[0] = y_ - exp(m*x_ + c);

        // 计算雅克比
        if (jacobians != NULL && jacobians[0] != NULL) {
            jacobians[0][0] = -x_*exp(m*x_ + c);
            jacobians[0][2] = -exp(m*x_ + c);
        }

        return true;
    }

private:
    const double x_;
    const double y_;
};

```

##  过参数化LocalParameterization

使用情况：

- 过参数化，比如四元数，传入参数是4个，实际参数是3个
- 其他空间上更新参数，eg：manifold space，tangent space

该类的内部主要接口

`LocalParameterization`类的作用是解决非线性优化中的过参数化问题。所谓过参数化，即待优化参数的实际自由度小于参数本身的自由度。例如在SLAM中，当采用四元数表示位姿时，由于四元数本身的约束（模长为1），**实际的自由度为3而非4**。此时，若直接传递四元数进行优化，冗余的维数会带来计算资源的浪费，需要使用`Ceres`预先定义的<font color=Crimson>`QuaternionParameterization`</font>对优化参数进行重构：

### 自定义LocalParameterization

<font color=Crimson>`LocalParaneterization`</font>本身是一个虚基类，详细定义如下。用户可以自行定义自己需要使用的子类，或使用Ceres预先定义好的子类。

```c++
class LocalParameterization {
 public:
  virtual ~LocalParameterization() {}
  //
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const = 0;//参数正切空间上的更新函数
  virtual bool ComputeJacobian(const double* x, double* jacobian) const = 0; //雅克比矩阵
  virtual bool MultiplyByJacobian(const double* x,
                                  const int num_rows,
                                  const double* global_matrix,
                                  double* local_matrix) const;//一般不用
  virtual int GlobalSize() const = 0; // 参数的实际维数
  virtual int LocalSize() const = 0; // 正切空间上的参数维数
};

```

上述成员函数中，需要我们改写的主要为`GlobalSize()`、`ComputeJacobian()`、`GlobalSize()`和`LocalSize()`，这里我们以ceres预先定义好的`QuaternionParameterization`为例具体说明，类声明如下：

```c++
class CERES_EXPORT QuaternionParameterization : public LocalParameterization {
 public:
  virtual ~QuaternionParameterization() {}
  virtual bool Plus(const double* x,
                    const double* delta,
                    double* x_plus_delta) const;
  virtual bool ComputeJacobian(const double* x,
                               double* jacobian) const;
  virtual int GlobalSize() const { return 4; }
  virtual int LocalSize() const { return 3; }
};

```

+ 可以看到，GlobalSize()的返回值为4，即四元数本身的实际维数；由于在内部优化时，ceres采用的是旋转矢量，维数为3，因此LocalSize()的返回值为3。

+ 重载的Plus函数给出了四元数的更新方法，接受参数分别为优化前的四元数x，用旋转矢量表示的增量delta，以及更新后的四元数x_plus_delta。函数首先将增量由旋转矢量转换为四元数，随后采用标准四元数乘法对四元数进行更新。

```c++
bool QuaternionParameterization::Plus(const double* x,
                                      const double* delta,
                                      double* x_plus_delta) const {
  // 将旋转矢量转换为四元数形式
  const double norm_delta =
      sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
  if (norm_delta > 0.0) {
    const double sin_delta_by_delta = (sin(norm_delta) / norm_delta);
    double q_delta[4];
    q_delta[0] = cos(norm_delta);
    q_delta[1] = sin_delta_by_delta * delta[0];
    q_delta[2] = sin_delta_by_delta * delta[1];
    q_delta[3] = sin_delta_by_delta * delta[2];
    // 采用四元数乘法更新
    QuaternionProduct(q_delta, x, x_plus_delta);
  } else {
    for (int i = 0; i < 4; ++i) {
      x_plus_delta[i] = x[i];
    }
  }
  return true;
}
```

+ `ComputeJacobian `函数给出了四元数相对于旋转矢量的雅克比矩阵计算方法，即 $\boldsymbol{J}_{4 \times 3}=\mathrm{d} \boldsymbol{q} / \mathrm{d} \boldsymbol{v}=$ $\mathrm{d}\left[\mathrm{q}_{\mathrm{w}}, \mathrm{q}_{\mathrm{x}}, \mathrm{q}_{\mathrm{y}}, \mathrm{q}_{\mathrm{z}}\right]^{\mathrm{T}} / \mathrm{d}[[\mathrm{x}, \mathrm{y}, \mathrm{z}]$ ，对应`Jacobian`维数为4行3列，存储方式为行主序。

```c++
bool QuaternionParameterization::ComputeJacobian(const double* x,
                                                 double* jacobian) const {
  jacobian[0] = -x[1]; jacobian[1]  = -x[2]; jacobian[2]  = -x[3];  // NOLINT
  jacobian[3] =  x[0]; jacobian[4]  =  x[3]; jacobian[5]  = -x[2];  // NOLINT
  jacobian[6] = -x[3]; jacobian[7]  =  x[0]; jacobian[8]  =  x[1];  // NOLINT
  jacobian[9] =  x[2]; jacobian[10] = -x[1]; jacobian[11] =  x[0];  // NOLINT
  return true;
}

```

##  **关于添加残差块，参数块，设置参数化的区别和调用关系**

### Problem::AddResidualBlock( )

`AddResidualBlock()`顾名思义主要用于向`Problem`类传递残差模块的信息，函数原型如下，传递的参数主要包括代价函数模块、损失函数模块和参数模块。

```c++
ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
										  LossFunction *loss_function, 
										  const vector<double *> parameter_blocks)
										  
ResidualBlockId Problem::AddResidualBlock(CostFunction *cost_function, 
										  LossFunction *loss_function,
										  double *x0, double *x1, ...)

```

+ 代价函数：包含了参数模块的维度信息，内部使用仿函数定义误差函数的计算方式。`AddResidualBlock( )`函数会检测传入的参数模块是否和代价函数模块中定义的维数一致，维度不一致时程序会强制退出。

- 损失函数：用于处理参数中含有野值的情况，避免错误量测对估计的影响，常用参数包括`HuberLoss`、`CauchyLoss`等（完整的参数列表参见Ceres API文档）；该参数可以取`NULL`或`nullptr`，此时损失函数为单位函数。

- 参数模块：待优化的参数，可一次性传入所有参数的指针容器vector<double*>或依次传入所有参数的指针double*。

### Problem::AddParameterBlock( )

用户在调用`AddResidualBlock( )`时其实已经隐式地向`Problem`传递了参数模块，但在一些情况下，需要用户显示地向Problem`传`入参数模块（通常出现在需要对优化参数进行重新参数化的情况）。`Ceres`提供了`Problem::AddParameterBlock( )`函数用于用户显式传递参数模块：

```c++
void Problem::AddParameterBlock(double *values, int size)

void Problem::AddParameterBlock(double *values, int size, LocalParameterization *local_parameterization)

```

其中，第一种函数原型除了会增加一些额外的参数检查之外，功能上和隐式传递参数并没有太大区别。第二种函数原型则会额外传入`LocalParameterization`参数，用于重构优化参数的维数

### 其他成员函数

`Probelm`还提供了其他关于`ResidualBlock`和`ParameterBlock`的函数，例如获取模块维数、判断是否存在模块、存在的模块数目等，这里只列出几个比较重要的函数，完整的列表参见[ceres API](http://www.ceres-solver.org/nnls_modeling.html#problem)：

```c++
// 设定对应的参数模块在优化过程中保持不变
void Problem::SetParameterBlockConstant(double *values)
// 设定对应的参数模块在优化过程中可变
void Problem::SetParameterBlockVariable(double *values)
// 设定优化下界
void Problem::SetParameterLowerBound(double *values, int index, double lower_bound)
// 设定优化上界
void Problem::SetParameterUpperBound(double *values, int index, double upper_bound)
// 该函数紧跟在参数赋值后，在给定的参数位置求解Problem，给出当前位置处的cost、梯度以及Jacobian矩阵；
bool Problem::Evaluate(const Problem::EvaluateOptions &options, 
					   double *cost, vector<double>* residuals, 
					   vector<double> *gradient, CRSMatrix *jacobian)

```



### 总结

- 参数正常更新，只需要调用`AddResidualBlock`
- 参数自定义更新，需要调用`AddParameterBlock`或者`SetParameterization`，要注意，数量一定要添加对，因为比如`vins-mono`里面参数非常多，搞不清楚参数维度就会很容易出错。·

