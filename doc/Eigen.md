## Eigen模块和头文件
Eigen库被分为一个Core模块和其他一些模块，每个模块有一些相应的头文件。 为了便于引用，Dense模块整合了一系列模块；Eigen模块整合了所有模块。一般情况下，include<Eigen/Dense> 就够了。



| Module      | Header file                 | Contents                                         |
| ----------- | --------------------------- | ------------------------------------------------ |
| Core        | #include<Eigen/Core>        | Matrix和Array类，基础的线性代数运算和数组操作    |
| Geometry    | #include<Eigen/Geometry>    | 旋转、平移、缩放、2维和3维的各种变换             |
| LU          | #include<Eigen/LU>          | 求逆，行列式，LU分解                             |
| Cholesky    | #include <Eigen/Cholesky>   | LLT和LDLT Cholesky分解                           |
| Householder | #include<Eigen/Householder> | 豪斯霍尔德变换，用于线性代数运算                 |
| SVD         | #include<Eigen/SVD>         | SVD分解                                          |
| QR          | #include<Eigen/QR>          | QR分解                                           |
| Eigenvalues | #include<Eigen/Eigenvalues> | 特征值，特征向量分解                             |
| Sparse      | #include<Eigen/Sparse>      | 稀疏矩阵的存储和一些基本的线性运算               |
| 稠密矩阵    | #include<Eigen/Dense>       | Core/Geometry/LU/Cholesky/SVD/QR/Eigenvalues模块 |
| 矩阵        | #include<Eigen/Eigen>       | 包括Dense和Sparse(整合库)                        |




## 		一.矩阵的使用
### 1.Matrix类
在Eigen，所有的矩阵和向量都是Matrix模板类的对象，Vector只是一种特殊的矩阵（一行或者一列）。

Matrix有6个模板参数，主要使用前三个参数，剩下的有默认值。
```c++
Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>
```
Scalar是表示元素的类型，RowsAtCompileTime为矩阵的行，ColsAtCompileTime为矩阵的列。

库中提供了一些类型便于使用，比如：
```c++
typedef Matrix<float, 4, 4> Matrix4f;
```
### 2.Vectors向量
列向量
```c++
typedef Matrix<float, 3, 1> Vector3f;
```
行向量
```c++
typedef Matrix<int, 1, 2> RowVector2i;
```
### 3.Dynamic
Eigen不只限于已知大小（编译阶段）的矩阵，有些矩阵的尺寸是运行时确定的，于是引入了一个特殊的标识符：Dynamic
```c++
typedef Matrix<double, Dynamic, Dynamic> MatrixXd;
typedef Matrix<int, Dynamic, 1> VectorXi;
Matrix<float, 3, Dynamic>
```
### 4.resizing
matrix的大小可以通过rows()、cols()、size()获取，resize()可以重新调整动态matrix的大小。
```c++
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
int main()
{
  MatrixXd m(2,5);
  m.resize(4,3);
  std::cout << "The matrix m is of size "
            << m.rows() << "x" << m.cols() << std::endl;//获得矩阵的行数，列数
  std::cout << "It has " << m.size() << " coefficients" << std::endl;//获得矩阵的大小
  VectorXd v(2);//动态向量
  v.resize(5);//动态调整向量的大小
  std::cout << "The vector v is of size " << v.size() << std::endl;
  std::cout << "As a matrix, v is of size "
            << v.rows() << "x" << v.cols() << std::endl;
}

```
输出：
```c++
The matrix m is of size 4x3
It has 12 coefficients
The vector v is of size 5
As a matrix, v is of size 5x1

```
如果matrix的实际大小不改变，resize函数不做任何操作。resize操作会执行析构函数：元素的值会被改变，如果不想改变执行 conservativeResize()。

为了统一API，所有的操作可用于指定大小的matrix，当然，实际中它不会改变大小。==尝试去改变一个固定大小的matrix到一个不同的值，会出发警告失败。只有如下是`合法的`。==
```c++
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
int main()
{
  Matrix4d m;
  m.resize(4,4); // no operation
  std::cout << "The matrix m is of size "
            << m.rows() << "x" << m.cols() << std::endl;
}
```
### 5.assignment 和 resizing
assignment（分配）是复制一个矩阵到另外一个，操作符=。Eigen会自动resize左变量大小等于右变量大小，比如：
```c++
MatrixXf a(2,2);
std::cout << "a is of size " << a.rows() << "x" << a.cols() << std::endl;
MatrixXf b(3,3);
a = b;
std::cout << "a is now of size " << a.rows() << "x" << a.cols() << std::endl;

a is of size 2x2
a is now of size 3x3

```
当然，如果左边量是固定大小的，上面的resizing是不允许的。
### 6.固定尺寸 vs 动态尺寸
实际中，应该使用固定尺寸还是动态尺寸，简单的答案是：小的尺寸用固定的，大的尺寸用动态的。使用固定尺寸可以避免动态内存的开辟，固定尺寸只是一个普通数组。

<font color='red'>**Matrix4f mymatrix;** </font>等价于<font color='red'> **float mymatrix[16];**</font>

<font color='red'>**MatrixXf mymatrix(rows,columns); **</font>等价于 float <font color='red'> \*mymatrix = new float[rows\*columns];</font>

使用固定尺寸(<=4*4)需要编译前知道矩阵大小，而且对于足够大的尺寸，如大于32，固定尺寸的收益可以忽略不计，而且可能导致栈崩溃。而且基于环境，Eigen会对动态尺寸做优化（类似于std::vector）
### 7. 定义Eigen

```c++
#include <iostream>
#include <Eigen/Dense>//引用Ｅｉｇｅｎ库
using namespace Eigen;
using namespace std;
template <typename T> using Mat2 = Matrix<T, 2 , 2>;//创建一个矩阵定义方式１
int main(int argc, char *argv[]){
    Mat2<int> mat1;
    mat1 << 1,1,
            1,1;//矩阵赋值
    cout << mat1 << endl;//输出矩阵的值
    Vector2d vec1;//创建一个矩阵定义方式２
    vec1 << 1 ,2 ;
    cout << vec1 << endl;
    MatrixXd mat(2,2);//创建一个矩阵定义方式３
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            mat(i , j) = 1;
            cout << mat(i,j)  << " ";
        }
        cout << endl;
    }
    Matrix<double,Dynamic,Dynamic> matri_xx;//创建一个动态矩阵
    return 0;
}

```
###  8. 全零阵，全一阵，单位阵和随机阵的定义方式
```c++
#include <iostream>
#include <eigen3/Eigen/Dense>//引用Ｅｉｇｅｎ库
using namespace Eigen;
using namespace std;
int main(int argc, char *argv[]){
    //全零阵，全一阵，单位阵和随机阵的定义方式
    MatrixXd matrix1 = MatrixXd::Zero(2,2);
    MatrixXd matrix2 = MatrixXd::Ones(2,2);
    MatrixXd matrix3 = MatrixXd::Identity(2,2);
    MatrixXd matrix4 = MatrixXd::Random(2,2);
    cout << matrix1 << endl;
    cout << matrix2 << endl;
    cout << matrix3 << endl;
    cout << matrix4 << endl;
    return 0;
    }
```

### 9. 矩阵的快速取元素操作
#### 9.1矩阵的快速取元素操作1
```c++
#include <iostream>
#include <eigen3/Eigen/Dense>//引用Ｅｉｇｅｎ库
using namespace Eigen;
using namespace std;
int main(int argc, char *argv[]){
    //矩阵的快速取元素操作
    ArrayXd vec(10);
    vec << 1,2,3,4,5,6,7,8,9,0;
    cout << vec <<endl;
    cout << vec.head(3) << endl;//前三个
    cout << vec.tail(4 ) << endl;//后几个
    cout << vec.segment(3,8) << endl;//中间的几个
    
    return 0;
}

```
#### 9.2 矩阵的快速取元素操作２
```c++
#include <iostream>
#include <eigen3/Eigen/Dense>//引用Ｅｉｇｅｎ库
using namespace Eigen;
using namespace std;
int main(int argc, char *argv[]){
    //矩阵的快速取元素操作
    MatrixXd mat(4,4);
    mat <<  1,2,3,4,
            5,6,7,8,
            9,10,11,12,
            13,14,15,16;
    cout << mat.col(2) << endl;
    cout << mat.row(2) << endl;
    //取矩阵中的某块
    cout << mat.block(1,1,2,2) << endl;//参考点行数，参考点列数，取行数的个数，取列数的个数
    cout << mat.block<2,2>(1,1) << endl;
    return 0;
}

```
### 10. 其他模板参数
上面只讨论了前三个参数，完整的模板参数如下：
```c++
Matrix<typename Scalar,
       int RowsAtCompileTime,
       int ColsAtCompileTime,
       int Options = 0,
       int MaxRowsAtCompileTime = RowsAtCompileTime,
       int MaxColsAtCompileTime = ColsAtCompileTime>
```
Options是一个比特标志位，这里，我们只介绍一种RowMajor，它表明matrix使用按行存储，默认是按列存储。<font color = 'fuchsia'>Matrix<float, 3, 3, RowMajor></font>

<font color = 'fuchsia'>MaxRowsAtCompileTime</font>和<font color = 'fuchsia'>MaxColsAtCompileTime</font>表示在编译阶段矩阵的上限。主要是避免动态内存分配，使用数组。

<font color = 'fuchsia'>Matrix<float, Dynamic, Dynamic, 0, 3, 4> </font>等价于 <font color = 'fuchsia'>float [12]</font>
<font color = 'fuchsia'>Matrix<float, Dynamic, Dynamic, 0, 3, 4> </font>等价于 <font color = 'fuchsia'>float [12]</font>

### 存储顺序
对于矩阵和二维数组有两种存储方式，列优先和行优先。
```c++
Matrix<int, 3, 4, ColMajor> Acolmajor;
Acolmajor << 8, 2, 2, 9,
             9, 1, 4, 4,
             3, 5, 4, 5;
cout << "The matrix A:" << endl;
cout << Acolmajor << endl << endl; 
cout << "In memory (column-major):" << endl;
for (int i = 0; i < Acolmajor.size(); i++)
  cout << *(Acolmajor.data() + i) << "  ";
cout << endl << endl;
Matrix<int, 3, 4, RowMajor> Arowmajor = Acolmajor;
cout << "In memory (row-major):" << endl;
for (int i = 0; i < Arowmajor.size(); i++)
 cout << *(Arowmajor.data() + i) << "  ";
 cout << endl;

```
```c++
The matrix A:
8 2 2 9
9 1 4 4
3 5 4 5

In memory (column-major):
8  9  3  2  1  5  2  4  4  9  4  5  

In memory (row-major):
8  2  2  9  9  1  4  4  3  5  4  5 

```
#### 存储顺序及选择
Matrix类模板中可以设定存储的方向，RowMajor表示行优先，ColMajor表示列优先。==默认是列优先==。

如何选择存储方式呢？

1. 如果要和其他库合作开发，为了转化方便，可以选择同样的存储方式。
2. 应用中涉及大量行遍历操作，应该选择行优先，寻址更快。反之亦然。
3. 默认是列优先，而且大多库都是按照这个顺序的，默认的不失为较好的。

## 二.矩阵和向量的运算
提供一些概述和细节：关于矩阵、向量以及标量的运算。
### 1.介绍
Eigen提供了matrix/vector的运算操作，既包括重载了c++的算术运算符+/-/*，也引入了一些特殊的运算比如点乘dot、叉乘cross等。

对于Matrix类（matrix和vectors）这些操作只支持线性代数运算，比如：matrix1*matrix2表示矩阵的乘积，<font color = 'red'>vetor+scalar是不允许的。</font>

### 2.加减
左右两侧变量具有相同的尺寸（行和列），并且元素类型相同（Eigen不自动转化类型）操作包括：

+ 二元运算 + 如a+b
+ 二元运算 - 如a-b
+ 一元运算 - 如-a
+ 复合运算 += 如a+=b
+ 复合运算 -= 如a-=b
```c++
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
int main()
{
  Matrix2d a;
  a << 1, 2,
       3, 4;
  MatrixXd b(2,2);
  b << 2, 3,
       1, 4;
  std::cout << "a + b =\n" << a + b << std::endl;
  std::cout << "a - b =\n" << a - b << std::endl;
  std::cout << "Doing a += b;" << std::endl;
  a += b;
  std::cout << "Now a =\n" << a << std::endl;
  Vector3d v(1,2,3);
  Vector3d w(1,0,0);
  std::cout << "-v + w - v =\n" << -v + w - v << std::endl;
}

```
输出结果：
```c++
a + b =
3 5
4 8
a - b =
-1 -1
 2  0
Doing a += b;
Now a =
3 5
4 8
-v + w - v =
-1
-4
-6

```
### 3.标量乘法和除法
乘/除标量是非常简单的，如下：

+ 二元运算 * 如matrix*scalar
+ 二元运算 * 如scalar*matrix
+ 二元运算 / 如matrix/scalar
+ 复合运算 *= 如matrix*=scalar
+ 复合运算 /= 如matrix/=scalar
```c++
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
int main()
{
  Matrix2d a;
  a << 1, 2,
       3, 4;
  Vector3d v(1,2,3);
  std::cout << "a * 2.5 =\n" << a * 2.5 << std::endl;
  std::cout << "0.1 * v =\n" << 0.1 * v << std::endl;
  std::cout << "Doing v *= 2;" << std::endl;
  v *= 2;
  std::cout << "Now v =\n" << v << std::endl;
}

```
输出：
```c++
a * 2.5 =
2.5   5
7.5  10
0.1 * v =
0.1
0.2
0.3
Doing v *= 2;
Now v =
2
4
6

```
### 3.表达式模板
在Eigen中，线性运算比如+不会对变量自身做任何操作，会返回一个“表达式对象”来描述被执行的计算。当整个表达式被评估完（一般是遇到=号），实际的操作才执行。

这样做主要是为了优化，比如:
```c++
VectorXf a(50), b(50), c(50), d(50);
...
a = 3*b + 4*c + 5*d;

```
Eigen会编译这段代码最终遍历一次即可运算完成。
```c++
for(int i = 0; i < 50; ++i)
  a[i] = 3*b[i] + 4*c[i] + 5*d[i];

```
因此，我们不必要担心大的线性表达式的运算效率。
### 5.转置和共轭
$a^{T}$ 表示transpose转置
$\bar{a}$ 表示conjugate共轭
$a^{*}$ 表示adjoint(共轭转置) 伴随矩阵

```c++
MatrixXcf a = MatrixXcf::Random(2,2);
cout << "Here is the matrix a\n" << a << endl;
cout << "Here is the matrix a^T\n" << a.transpose() << endl;
cout << "Here is the conjugate of a\n" << a.conjugate() << endl;
cout << "Here is the matrix a^*\n" << a.adjoint() << endl;

```
输出：
```c++
Here is the matrix a
 (-0.211,0.68) (-0.605,0.823)
 (0.597,0.566)  (0.536,-0.33)
Here is the matrix a^T
 (-0.211,0.68)  (0.597,0.566)
(-0.605,0.823)  (0.536,-0.33)
Here is the conjugate of a
 (-0.211,-0.68) (-0.605,-0.823)
 (0.597,-0.566)    (0.536,0.33)
Here is the matrix a^*
 (-0.211,-0.68)  (0.597,-0.566)
(-0.605,-0.823)    (0.536,0.33)

```
对于实数矩阵，conjugate不执行任何操作，adjoint等价于transpose。

transpose和adjoint会简单的返回一个代理对象并不对本省做转置。如果执行<font color = fuchsia> b=a.transpose() </font>，a不变，转置结果被赋值给b。如果执行 <font color = fuchsia>a=a.transpose() </font>Eigen在转置结束之前结果会开始写入a，所以a的最终结果不一定等于a的转置。
这被称为“别名问题”。在debug模式，当assertions打开的情况加，这种常见陷阱可以被自动检测到。
```c++
Matrix2i a; a << 1, 2, 3, 4;
cout << "Here is the matrix a:\n" << a << endl;
a = a.transpose(); // !!! do NOT do this !!!
cout << "and the result of the aliasing effect:\n" << a << endl;

Here is the matrix a:
1 2
3 4
and the result of the aliasing effect:
1 2
2 4

```
### 6.矩阵-矩阵的乘法和矩阵-向量的乘法
向量也是一种矩阵，实质都是矩阵-矩阵的乘法。

+二元运算 *如a*b
+复合运算 *=如a*=b
```c++
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
int main()
{
  Matrix2d mat;
  mat << 1, 2,
         3, 4;
  Vector2d u(-1,1), v(2,0);
  std::cout << "Here is mat*mat:\n" << mat*mat << std::endl;//矩阵*矩阵
  std::cout << "Here is mat*u:\n" << mat*u << std::endl;//矩阵*向量
  std::cout << "Here is u^T*mat:\n" << u.transpose()*mat << std::endl;//向量的转置*矩阵
  std::cout << "Here is u^T*v:\n" << u.transpose()*v << std::endl;//向量的转置*向量
  std::cout << "Here is u*v^T:\n" << u*v.transpose() << std::endl;//向量*向量的转置
  std::cout << "Let's multiply mat by itself" << std::endl;
  mat = mat*mat;
  std::cout << "Now mat is mat:\n" << mat << std::endl;
}

```
输出：
```c++
Here is mat*mat:
 7 10
15 22
Here is mat*u:
1
1
Here is u^T*mat:
2 2
Here is u^T*v:
-2
Here is u*v^T:
-2 -0
 2  0
Let's multiply mat by itself
Now mat is mat:
 7 10
15 22

```
### 7.点积运算和叉运算
dot()执行点积，cross()执行叉积，点运算得到1*1的矩阵。当然，点运算也可以用u.adjoint()*v来代替。
```c++
#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
int main()
{
  Vector3d v(1,2,3);
  Vector3d w(0,1,2);
  cout << "Dot product: " << v.dot(w) << endl;
  double dp = v.adjoint()*w; // automatic conversion of the inner product to a scalar
  cout << "Dot product via a matrix product: " << dp << endl;
  cout << "Cross product:\n" << v.cross(w) << endl;
}

```
### 8.基础的归约操作
Eigen提供了而一些归约函数：sum()、prod()、maxCoeff()和minCoeff()，他们对所有元素进行操作。
```c++
#include <iostream>
#include <Eigen/Dense>
using namespace std;
int main()
{
  Eigen::Matrix2d mat;
  mat << 1, 2,
         3, 4;
  cout << "Here is mat.sum():       " << mat.sum()       << endl;//矩阵求和
  cout << "Here is mat.prod():      " << mat.prod()      << endl;//矩阵元素相乘
  cout << "Here is mat.mean():      " << mat.mean()      << endl;//求取矩阵元素均值
  cout << "Here is mat.minCoeff():  " << mat.minCoeff()  << endl;//输出最小元素，也可以获取元素位置
  cout << "Here is mat.maxCoeff():  " << mat.maxCoeff()  << endl;//输出最大元素，也可以获取元素位置
  cout << "Here is mat.trace():     " << mat.trace()     << endl;//输出矩阵的迹
}

```
```c++
Here is mat.sum():       10
Here is mat.prod():      24
Here is mat.mean():      2.5
Here is mat.minCoeff():  1
Here is mat.maxCoeff():  4
Here is mat.trace():     5
```
### 9. 范数计算
L2范数``` squareNorm()```，等价于计算```vector```的自身点积，```norm()```返回```squareNorm```的开方根。

这些操作应用于```matrix，norm() ```会返回```Frobenius```或```Hilbert-Schmidt```范数。

如果你想使用其他Lp范数，可以使用```lpNorm< p >()```方法。p可以取```Infinity，表示L∞范数。```

```c++

int main()
{
  VectorXf v(2);
  MatrixXf m(2,2), n(2,2);
  
  v << -1,
       2;
  
  m << 1,-2,
       -3,4;
  cout << "v.squaredNorm() = " << v.squaredNorm() << endl;//L2范数
  cout << "v.norm() = " << v.norm() << endl;//模
  cout << "v.lpNorm<1>() = " << v.lpNorm<1>() << endl;//L1范数
  cout << "v.lpNorm<Infinity>() = " << v.lpNorm<Infinity>() << endl;//L∞范数
  cout << endl;
  cout << "m.squaredNorm() = " << m.squaredNorm() << endl;
  cout << "m.norm() = " << m.norm() << endl;
  cout << "m.lpNorm<1>() = " << m.lpNorm<1>() << endl;
  cout << "m.lpNorm<Infinity>() = " << m.lpNorm<Infinity>() << endl;
}

```
输出：
```c++
v.squaredNorm() = 5
v.norm() = 2.23607
v.lpNorm<1>() = 3
v.lpNorm<Infinity>() = 2

m.squaredNorm() = 30
m.norm() = 5.47723
m.lpNorm<1>() = 10
m.lpNorm<Infinity>() = 4

```



### 10. 操作的有效性:
Eigen会检测执行操作的有效性，在编译阶段Eigen会检测它们，错误信息是繁冗的，但错误信息会大写字母突出，比如:
```c++
Matrix3f m;
Vector4f v;
v = m*v;      // Compile-time error: YOU_MIXED_MATRICES_OF_DIFFERENT_SIZES
```
当然动态尺寸的错误要在运行时发现，如果在debug模式，assertions会触发后，程序将崩溃。
```c++
MatrixXf m(3,3);
VectorXf v(4);
v = m * v; // Run-time assertion failure here: "invalid matrix product"
```
## 三.Array类和元素级操作
### 1. 为什么使用Array
相对于Matrix提供的线性代数运算，Array类提供了更为一般的数组功能。Array类为<font color = 'fuchsia'>元素级的操作</font>提供了有效途径，比如<font color = 'fuchsia'>点加（每个元素加值）</font>或<font color = 'fuchsia'>两个数据==相应元素==的点乘</font>。

Array是个类模板（类似于Matrx）,前三个参数是必须指定的，后三个是可选的，这点和Matrix是相同的。
```c++
Array<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime>

```
Eigen也提供的一些常用类定义，Array是同时支持一维和二维的（Matrix二维，Vector一维）

| Type                          | Tyoedef  |
| ----------------------------- | -------- |
| Array<float,Dynamic,1>        | ArrayXf  |
| Array<float,3,1>              | Array3f  |
| Array<double,Dynamic,Dynamic> | ArrayXXd |
| Array<double,3,3>             | Array33d |
### 2. 获取元素
读写操作重载于matrix，  <<   可以用于初始化array或打印。
```c++
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
int main()
{
  ArrayXXf  m(2,2);
  
  // assign some values coefficient by coefficient
  m(0,0) = 1.0; m(0,1) = 2.0;
  m(1,0) = 3.0; m(1,1) = m(0,1) + m(1,0);
  
  // print values to standard output
  cout << m << endl << endl;
 
  // using the comma-initializer is also allowed
  m << 1.0,2.0,
       3.0,4.0;
     
  // print values to standard output
  cout << m << endl;
}

```
### 3.加法和减法
和matrix类似，要求array的尺寸一致。同时支持<font color = 'red'>array+/-scalar</font>的操作！
```c++
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
int main()
{
  ArrayXXf a(3,3);
  ArrayXXf b(3,3);
  a << 1,2,3,
       4,5,6,
       7,8,9;
  b << 1,2,3,
       1,2,3,
       1,2,3;
       
  // Adding two arrays
  cout << "a + b = " << endl << a + b << endl << endl;
  // Subtracting a scalar from an array
  cout << "a - 2 = " << endl << a - 2 << endl;
}

```
### 4. 乘法
支持array*scalar（类似于matrix），但是当执行array*array时，执行的是相应元素的乘积，因此两个array必须具有<font color = 'red'>相同的尺寸</font>。
```c++
int main()
{
  ArrayXXf a(2,2);
  ArrayXXf b(2,2);
  a << 1,2,
       3,4;
  b << 5,6,
       7,8;
  cout << "a * b = " << endl << a * b << endl;
}

a * b = 
 5 12
21 32

```
### 5. 其他元素级操作

| **Function** | **function**              |
| ------------ | ------------------------- |
| abs          | 绝对值                    |
| sqrt         | 平方根                    |
| min(.)       | 两个array相应元素的最小值 |
```c++
int main()
{
  ArrayXf a = ArrayXf::Random(5);
  a *= 2;
  cout << "a =" << endl 
       << a << endl;
  cout << "a.abs() =" << endl 
       << a.abs() << endl;
  cout << "a.abs().sqrt() =" << endl 
       << a.abs().sqrt() << endl;
  cout << "a.min(a.abs().sqrt()) =" << endl 
       << a.min(a.abs().sqrt()) << endl;
}

```
### 6. array和matrix之间的转换
当需要线性代数类操作时，请使用Matrix；但需要元素级操作时，需要使用Array。这样就需要提供两者的转化方法。

Matrix提供了.array()函数将它们转化为Array对象。

Array提供了.matrix()函数将它们转化为Matrix对象。

在Eigen，在表达式中混合Matrix和Array操作是被禁止的，但是可以将array表达式结果赋值为matrix。

另外，Matrix提供了cwiseProduct函数也实现了点乘。
```c++
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
int main()
{
  MatrixXf m(2,2);
  MatrixXf n(2,2);
  MatrixXf result(2,2);
  m << 1,2,
       3,4;
  n << 5,6,
       7,8;
  result = m * n;
  cout << "-- Matrix m*n: --" << endl << result << endl << endl;
  result = m.array() * n.array();
  cout << "-- Array m*n: --" << endl << result << endl << endl;//点乘
  result = m.cwiseProduct(n);
  cout << "-- With cwiseProduct: --" << endl << result << endl << endl;//点乘
  result = m.array() + 4;//matrix不支持这种运算
  cout << "-- Array m + 4: --" << endl << result << endl << endl;
}

```
```c++
-- Matrix m*n: --
19 22
43 50

-- Array m*n: --
 5 12
21 32

-- With cwiseProduct: --
 5 12
21 32

-- Array m + 4: --
5 6
7 8

```
## 四. 块操作
块是matrix或array中的矩形子部分。
### 1. 使用块
函数.block()，有两种形式

| operation            | 构建一个动态尺寸的block | 构建一个固定尺寸的block |
| -------------------- | ----------------------- | ----------------------- |
| 起点(i,j)块大小(p,q) | .block(i,j,p,q)         | .block< p,q >(i,j)      |

Eigen中，索引从0开始。

两个版本都可以用于固定尺寸和动态尺寸的matrix/array。功能是等价的，只是固定尺寸的版本在block较小时速度更快一些。
```c++
int main()
{
  Eigen::MatrixXf m(4,4);
  m <<   1, 2, 3, 4,
               5, 6, 7, 8,
              9,10,11,12,
             13,14,15,16;
  cout << "Block in the middle" << endl;
  cout << m.block<2,2>(1,1) << endl << endl;
  for (int i = 1; i <= 3; ++i)
  {
    cout << "Block of size " << i << "x" << i << endl;
    cout << m.block(0,0,i,i) << endl << endl;
  }
}

```
输出:
```c++
Block in the middle
 6  7
10 11

Block of size 1x1
1

Block of size 2x2
1 2
5 6

Block of size 3x3
 1  2  3
 5  6  7
 9 10 11

```
```c++
int main()
{
  Array22f m;
  m << 1,2,
       3,4;
  Array44f a = Array44f::Constant(0.6);
  cout << "Here is the array a:" << endl << a << endl << endl;
  a.block<2,2>(1,1) = m;
  cout << "Here is now a with m copied into its central 2x2 block:" << endl << a << endl << endl;
  a.block(0,0,2,3) = a.block(2,1,2,3);
  cout << "Here is now a with bottom-right 2x3 block copied into top-left 2x2 block:" << endl << a << endl << endl;
}
```
输出
```c++
Here is the array a:
0.6 0.6 0.6 0.6
0.6 0.6 0.6 0.6
0.6 0.6 0.6 0.6
0.6 0.6 0.6 0.6

Here is now a with m copied into its central 2x2 block:
0.6 0.6 0.6 0.6
0.6   1   2 0.6
0.6   3   4 0.6
0.6 0.6 0.6 0.6

Here is now a with bottom-right 2x3 block copied into top-left 2x2 block:
  3   4 0.6 0.6
0.6 0.6 0.6 0.6
0.6   3   4 0.6
0.6 0.6 0.6 0.6

```
### 2. 行和列
```c++
int main()
{
  Eigen::MatrixXf m(3,3);
  m << 1,2,3,
       4,5,6,
       7,8,9;
  cout << "Here is the matrix m:" << endl << m << endl;
  cout << "2nd Row: " << m.row(1) << endl;
  m.col(2) += 3 * m.col(0);
  cout << "After adding 3 times the first column into the third column, the matrix m is:\n";
  cout << m << endl;
}

```
输出：
```c++
Here is the matrix m:
1 2 3
4 5 6
7 8 9
2nd Row: 4 5 6
After adding 3 times the first column into the third column, the matrix m is:
 1  2  6
 4  5 18
 7  8 30

```
### 3. 角相关操作

| operation  | dynamic-size block             | fixed-size block                   |
| ---------- | ------------------------------ | ---------------------------------- |
| 左上角p\*q | matrix.topLeftCorner(p,q);     | matrix.topLeftCorner< p,q >();     |
| 左下角p\*q | matrix.bottomLeftCorner(p,q);  | matrix.bottomLeftCorner< p,q >();  |
| 右上角p\*q | matrix.topRightCorner(p,q);    | matrix.topRightCorner< p,q >();    |
| 右下角p\*q | matrix.bottomRightCorner(p,q); | matrix.bottomRightCorner< p,q >(); |
| 前q行      | matrix.topRows(q);             | matrix.topRows< q >();             |
| 后q行      | matrix.bottomRows(q);          | matrix.bottomRows< q >();          |
| 左p列      | matrix.leftCols(p);            | matrix.leftCols< p >();            |
| 右p列      | matrix.rightCols(p);           | matrix.rightCols< p >();           |
```c++
int main()
{
  Eigen::Matrix4f m;
  m << 1, 2, 3, 4,
       5, 6, 7, 8,
       9, 10,11,12,
       13,14,15,16;
  cout << "m.leftCols(2) =" << endl << m.leftCols(2) << endl << endl;
  cout << "m.bottomRows<2>() =" << endl << m.bottomRows<2>() << endl << endl;
  m.topLeftCorner(1,3) = m.bottomRightCorner(3,1).transpose();
  cout << "After assignment, m = " << endl << m << endl;
}
```
输出：
```c++
m.leftCols(2) =
 1  2
 5  6
 9 10
13 14

m.bottomRows<2>() =
 9 10 11 12
13 14 15 16

After assignment, m = 
 8 12 16  4
 5  6  7  8
 9 10 11 12
13 14 15 16

```
### 4. vectors的块操作

| **operation**  | **dynamic-size block** | **fixed-size block**    |
| -------------- | ---------------------- | ----------------------- |
| 前n个          | vector.head(n);        | vector.head< n >();     |
| 后n个          | vector.tail(n);        | vector.tail< n >();     |
| i起始的n个元素 | vector.segment(i,n);   | vector.segment< n >(i); |

## 五. 原生缓存的接口：Map类
Map的定义
```c++
Map<Matrix<typename Scalar, int RowsAtCompileTime, int ColsAtCompileTime> >
```
默认情况下，Map只需要一个模板参数。

为了构建Map变量，我们需要其余的两个信息：一个指向元素数组的指针，Matrix/vector的尺寸。定义一个float类型的矩阵：<font color = 'red'>```c++  Map<MatrixXf> mf(pf,rows,columns);```</font> pf是一个数组指针float 

固定尺寸的整形vector声明： ```Map<const Vector4i> mi(pi);```

注意:Map没有默认的构造函数，你需要传递一个指针来初始化对象。

Mat是灵活地足够去容纳多种不同的数据表示，其他的两个模板参数：
```c++
Map<typename MatrixType,
    int MapOptions,
    typename StrideType>
```
```MapOptions```标识指针是否是对齐的（Aligned），默认是Unaligned。

```StrideType```表示内存数组的组织方式：行列的步长。
```c++
int array[8];
for(int i = 0; i < 8; ++i) array[i] = i;
cout << "Column-major:\n" << Map<Matrix<int,2,4> >(array) << endl;
cout << "Row-major:\n" << Map<Matrix<int,2,4,RowMajor> >(array) << endl;
cout << "Row-major using stride:\n" <<
Map<Matrix<int,2,4>, Unaligned, Stride<1,4> >(array) << endl;

```
输出
```c++
Column-major:
0 2 4 6
1 3 5 7
Row-major:
0 1 2 3
4 5 6 7
Row-major using stride:
0 1 2 3
4 5 6 7

```
### 使用Map变量 
可以像Eigen的其他类型一样来使用Map类型。
```c++
typedef Matrix<float,1,Dynamic> MatrixType;
typedef Map<MatrixType> MapType;
typedef Map<const MatrixType> MapTypeConst;   // a read-only map
const int n_dims = 5;
  
MatrixType m1(n_dims), m2(n_dims);
m1.setRandom();
m2.setRandom();
float *p = &m2(0);  // get the address storing the data for m2
MapType m2map(p,m2.size());   // m2map shares data with m2
MapTypeConst m2mapconst(p,m2.size());  // a read-only accessor for m2
cout << "m1: " << m1 << endl;
cout << "m2: " << m2 << endl;
cout << "Squared euclidean distance: " << (m1-m2).squaredNorm() << endl;
cout << "Squared euclidean distance, using map: " <<
  (m1-m2map).squaredNorm() << endl;
m2map(3) = 7;   // this will change m2, since they share the same array
cout << "Updated m2: " << m2 << endl;
cout << "m2 coefficient 2, constant accessor: " << m2mapconst(2) << endl;
/* m2mapconst(2) = 5; */   // this yields a compile-time error

```
输出：
```c++
m1:   0.68 -0.211  0.566  0.597  0.823
m2: -0.605  -0.33  0.536 -0.444  0.108
Squared euclidean distance: 3.26
Squared euclidean distance, using map: 3.26
Updated m2: -0.605  -0.33  0.536      7  0.108
m2 coefficient 2, constant accessor: 0.536
//Eigen提供的函数都兼容Map对象。
```
## Eigen中基本和常用函数
### Eigen 中矩阵的定义

```c++
#include <Eigen/Dense>                  // 基本函数只需要包含这个头文件
Matrix<double, 3, 3> A;                 // 固定了行数和列数的矩阵和Matrix3d一致.
Matrix<double, 3, Dynamic> B;           // 固定行数.
Matrix<double, Dynamic, Dynamic> C;     // 和MatrixXd一致.
Matrix<double, 3, 3, RowMajor> E;       // 按行存储; 默认按列存储.
Matrix3f P, Q, R;                       // 3x3 float 矩阵.
Vector3f x, y, z;                       // 3x1 float 列向量.
RowVector3f a, b, c;                    // 1x3 float 行向量.
VectorXd v;                             // 动态长度double型列向量
// Eigen          // Matlab             // comments
x.size()          // length(x)          // 向量长度
C.rows()          // size(C,1)          // 矩阵行数
C.cols()          // size(C,2)          // 矩阵列数
x(i)              // x(i+1)             // 下标0开始
C(i,j)            // C(i+1,j+1)         // 下标0开始

```



### Eigen 中矩阵的使用方法

```c++
A.resize(4, 4);   // 如果越界触发运行时错误.
B.resize(4, 9);   // 如果越界触发运行时错误.
A.resize(3, 3);   // Ok; 没有越界.
B.resize(3, 9);   // Ok; 没有越界.

A << 1, 2, 3,     // Initialize A. The elements can also be
     4, 5, 6,     // matrices, which are stacked along cols
     7, 8, 9;     // and then the rows are stacked.
B << A, A, A;     // B is three horizontally stacked A's.   三行A
A.fill(10);       // Fill A with all 10's.                  全10

```



### Eigen 中常用矩阵生成

```c++
// Eigen                            // Matlab
MatrixXd::Identity(rows,cols)       // eye(rows,cols) 单位矩阵
C.setIdentity(rows,cols)            // C = eye(rows,cols) 单位矩阵
MatrixXd::Zero(rows,cols)           // zeros(rows,cols) 零矩阵
C.setZero(rows,cols)                // C = ones(rows,cols) 零矩阵
MatrixXd::Ones(rows,cols)           // ones(rows,cols)全一矩阵
C.setOnes(rows,cols)                // C = ones(rows,cols)全一矩阵
MatrixXd::Random(rows,cols)         // rand(rows,cols)*2-1        // 元素随机在-1->1
C.setRandom(rows,cols)              // C = rand(rows,cols)*2-1 同上
VectorXd::LinSpaced(size,low,high)  // linspace(low,high,size)'线性分布的数组
v.setLinSpaced(size,low,high)       // v = linspace(low,high,size)'线性分布的数组

```



### Eigen 中矩阵分块

```c++
// Eigen                           // Matlab
x.head(n)                          // x(1:n)    用于数组提取前n个[vector]
x.head<n>()                        // x(1:n)    同理
x.tail(n)                          // x(end - n + 1: end)同理
x.tail<n>()                        // x(end - n + 1: end)同理
x.segment(i, n)                    // x(i+1 : i+n)同理
x.segment<n>(i)                    // x(i+1 : i+n)同理
P.block(i, j, rows, cols)          // P(i+1 : i+rows, j+1 : j+cols)i,j开始，rows行cols列
P.block<rows, cols>(i, j)          // P(i+1 : i+rows, j+1 : j+cols)i,j开始，rows行cols列
P.row(i)                           // P(i+1, :)i行
P.col(j)                           // P(:, j+1)j列
P.leftCols<cols>()                 // P(:, 1:cols)左边cols列
P.leftCols(cols)                   // P(:, 1:cols)左边cols列
P.middleCols<cols>(j)              // P(:, j+1:j+cols)中间从j数cols列
P.middleCols(j, cols)              // P(:, j+1:j+cols)中间从j数cols列
P.rightCols<cols>()                // P(:, end-cols+1:end)右边cols列
P.rightCols(cols)                  // P(:, end-cols+1:end)右边cols列
P.topRows<rows>()                  // P(1:rows, :)同列
P.topRows(rows)                    // P(1:rows, :)同列
P.middleRows<rows>(i)              // P(i+1:i+rows, :)同列
P.middleRows(i, rows)              // P(i+1:i+rows, :)同列
P.bottomRows<rows>()               // P(end-rows+1:end, :)同列
P.bottomRows(rows)                 // P(end-rows+1:end, :)同列
P.topLeftCorner(rows, cols)        // P(1:rows, 1:cols)上左角rows行，cols列
P.topRightCorner(rows, cols)       // P(1:rows, end-cols+1:end)上右角rows行，cols列
P.bottomLeftCorner(rows, cols)     // P(end-rows+1:end, 1:cols)下左角rows行，cols列
P.bottomRightCorner(rows, cols)    // P(end-rows+1:end, end-cols+1:end)下右角rows行，cols列
P.topLeftCorner<rows,cols>()       // P(1:rows, 1:cols)同上
P.topRightCorner<rows,cols>()      // P(1:rows, end-cols+1:end)同上
P.bottomLeftCorner<rows,cols>()    // P(end-rows+1:end, 1:cols)同上
P.bottomRightCorner<rows,cols>()   // P(end-rows+1:end, end-cols+1:end)同上

```

### Eigen 中矩阵元素交换
```c++
// Eigen                           // Matlab
R.row(i) = P.col(j);               // R(i, :) = P(:, i)交换列为行
R.col(j1).swap(mat1.col(j2));      // R(:, [j1 j2]) = R(:, [j2, j1]) 交换列

```
### Eigen 中矩阵转置
```c++
// Views, transpose, etc; all read-write except for .adjoint().
// Eigen                           // Matlab
R.adjoint()                        // R' 伴随矩阵
R.transpose()                      // R.' or conj(R')转置
R.diagonal()                       // diag(R)对角
x.asDiagonal()                     // diag(x)对角阵(没有重载<<)
R.transpose().colwise().reverse(); // rot90(R)所有元素逆时针转了90度
R.conjugate()                      // conj(R)共轭矩阵

```
### Eigen 中矩阵乘积
```c++
// 与Matlab一致, 但是matlab不支持*=等形式的运算.
// Matrix-vector.  Matrix-matrix.   Matrix-scalar.
y  = M*x;          R  = P*Q;        R  = P*s;
a  = b*M;          R  = P - Q;      R  = s*P;
a *= M;            R  = P + Q;      R  = P/s;
                   R *= Q;          R  = s*P;
                   R += Q;          R *= s;
                   R -= Q;          R /= s;

```
### Eigen 中矩阵元素操作
```c++
// Vectorized operations on each element independently
// Eigen                  // Matlab
R = P.cwiseProduct(Q);    // R = P .* Q 对应点相乘
R = P.array() * s.array();// R = P .* s 对应点相乘
R = P.cwiseQuotient(Q);   // R = P ./ Q 对应点相除
R = P.array() / Q.array();// R = P ./ Q对应点相除
R = P.array() + s.array();// R = P + s对应点相加
R = P.array() - s.array();// R = P - s对应点相减
R.array() += s;           // R = R + s全加s
R.array() -= s;           // R = R - s全减s
R.array() < Q.array();    // R < Q 以下的都是针对矩阵的单个元素的操作
R.array() <= Q.array();   // R <= Q矩阵元素比较，会在相应位置置0或1
R.cwiseInverse();         // 1 ./ P
R.array().inverse();      // 1 ./ P
R.array().sin()           // sin(P) 
R.array().cos()           // cos(P)
R.array().pow(s)          // P .^ s
R.array().square()        // P .^ 2
R.array().cube()          // P .^ 3
R.cwiseSqrt()             // sqrt(P)
R.array().sqrt()          // sqrt(P)
R.array().exp()           // exp(P)
R.array().log()           // log(P)
R.cwiseMax(P)             // max(R, P) 对应取大
R.array().max(P.array())  // max(R, P) 对应取大
R.cwiseMin(P)             // min(R, P) 对应取小
R.array().min(P.array())  // min(R, P) 对应取小
R.cwiseAbs()              // abs(P) 绝对值
R.array().abs()           // abs(P) 绝对值
R.cwiseAbs2()             // abs(P.^2) 绝对值平方
R.array().abs2()          // abs(P.^2) 绝对值平方
(R.array() < s).select(P,Q);  // (R < s ? P : Q)这个也是单个元素的操作

```
### Eigen中矩阵归约操作
```c++
// Reductions.
int r, c;
// Eigen                  // Matlab
R.minCoeff()              // min(R(:))最小值
R.maxCoeff()              // max(R(:))最大值
s = R.minCoeff(&r, &c)    // [s, i] = min(R(:)); [r, c] = ind2sub(size(R), i);
s = R.maxCoeff(&r, &c)    // [s, i] = max(R(:)); [r, c] = ind2sub(size(R), i);
R.sum()                   // sum(R(:))求和
R.colwise().sum()         // sum(R)列求和1×N
R.rowwise().sum()         // sum(R, 2) or sum(R')'行求和N×1
R.prod()                  // prod(R(:))所有乘积
R.colwise().prod()        // prod(R)列乘积
R.rowwise().prod()        // prod(R, 2) or prod(R')'行乘积
R.trace()                 // trace(R)迹
R.all()                   // all(R(:))且运算
R.colwise().all()         // all(R) 且运算
R.rowwise().all()         // all(R, 2) 且运算
R.any()                   // any(R(:)) 或运算
R.colwise().any()         // any(R) 或运算
R.rowwise().any()         // any(R, 2) 或运算
```
### Eigen 中矩阵特征值
```c++
// Eigen                          // Matlab
A.eigenvalues();                  // eig(A);特征值
EigenSolver<Matrix3d> eig(A);     // [vec val] = eig(A)
eig.eigenvalues();                // diag(val)与前边的是一样的结果
eig.eigenvectors();               // vec 特征值对应的特征向量
```
###	Eigen中矩阵类型转换
```c++
 //Type conversion
// Eigen                           // Matlab
A.cast<double>();                  // double(A)
A.cast<float>();                   // single(A)
A.cast<int>();                     // int32(A) 向下取整
A.real();                          // real(A)
A.imag();                          // imag(A)
// if the original type equals destination type, no work is done

```
## 六. 求解Ax = b

![img](https://images2018.cnblogs.com/blog/1135655/201803/1135655-20180305185044796-1767709178.png)

###    1. LU分解

在线性代数中， LU分解(LU Decomposition)是矩阵分解的一种，可以将一个矩阵分解为一个单位下三角矩阵和一个上三角矩阵的乘积（有时是它们和一个置换矩阵的乘积）。LU分解主要应用在数值分析中，用来解线性方程、求反矩阵或计算行列式。
本质上，LU分解是高斯消元的一种表达方式。首先，对矩阵A通过初等行变换将其变为一个上三角矩阵。对于学习过线性代数的同学来说，这个过程应该很熟悉，线性代数考试中求行列式求逆一般都是通过这种方式来求解。然后，将原始矩阵A变为上三角矩阵的过程，对应的变换矩阵为一个下三角矩阵。这中间的过程，就是Doolittle algorithm(杜尔里特算法)。
 ![img](https://img-blog.csdn.net/20171117113045370)
**下面是关于使用LU分解求解方程组Ax = b：**
当系数**矩阵A完成了LU分解**后，方程组Ax = b就可以化为L(Ux) = b，等价于求解两个方程组Ly = b和Ux = y；

### 2. QR分解
QR 分解是三种将矩阵分解的方式之一。这种方式，把矩阵分解成一个正交矩阵Q与一个上三角矩阵R的积。

![在这里插入图片描述](https://img-blog.csdnimg.cn/20190401155805955.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dlaXhpbl80MjY5MTc1Mg==,size_16,color_FFFFFF,t_70)

对于Ax=b，直接求A的逆有时过于复杂，因此使用QR分解算法：

![在这里插入图片描述](https://img-blog.csdnimg.cn/20190401155751621.png)
### 3. LDLT分解（LU的进一步分解）
对称矩阵A可以分解成一个下三角矩阵L（Lower下）和一个对角矩阵D（Diagonal对角线）以及一个下三角矩阵L的转置LT三个矩阵相乘的形式。如下式 ![img](https://img-blog.csdn.net/20180505222147238?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dhbmdzaHVhaWxwcA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
由A的分解可知$A^{T}=A$，即A的转置等于A矩阵本身
$LDL^T$分解解满秩最小二乘问题：
一般无法得到满足对称矩阵A，因此需要使对$A^{T}A$（满足对称）进行分解。将求解问题转换成下面的式子: $A^{T}AX = A^{T}b$

>定理1： $ A^{T}A$不一定是正定矩阵，但一定是半正定矩阵（即特征值一定大于等于0）
>定理2：$ A^{T}A$是正定矩阵的充分必要条件是 A 满足列满秩

由于r(A)=n，所以$A^{T}A$是对称（正定）矩阵，式子有唯一解，使用$LDL^{T}$分解的步骤是：
（1）定义矩阵$C=A^{T}*A$，$d=A^{T}*b$；
（2）对C进行$LDL^{T}$分解$C=L*D*L^{T}$，原式变成$L*D*L^{T}*x=d$
（3）令$y=L^{T}*x$，原式变成$L*D*y=d$,求解此方程得到y，然后求解$y=L^{T}*x$得到x。
$LDL^{T}$分解速度要快于QR分解。


> 补充:正定矩阵(PD):
>  给定一个大小为 $n\times n$ 的实对称矩阵 A ，若对于任意长度为 n 的非零向量 X，有$X^{T}AX > 0$恒成立，则矩阵 A   是一个正定矩阵。
> 半正定矩阵(PSD)
> 给定一个大小为 $n\times n$ 的实对称矩阵 A ，若对于任意长度为 n 的非零向量 X，有 $X^{T}AX>=0$恒成立, 则矩阵 A 是一个半正定矩阵。

### 4. Cholesky分解法(LLT分解)
Cholesky分解是LDLT分解的一种特殊形式，也就是其中的D是单位矩阵。正定对称矩阵 A可以分解成一个下三角矩阵L和这个下三角矩阵L的转置LT相乘的形式。如下式 

![这里写图片描述](https://img-blog.csdn.net/20180505222350211?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dhbmdzaHVhaWxwcA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

&emsp;Cholesky分解法又叫平方根法，是求解对称正定线性方程组最常用的方法之一。对于一般矩阵，为了消除LU分解的局限性和误差的过分积累，采用了选主元的方法，但对于对称正定矩阵而言，选主元是不必要的。

定理：若对称$A\in{R^{n\times n}}$正定，则存在一个对角元为正数的下三角矩阵$L\in{R^{n\times n}}$，使得成立$A = LL^{T}$成立

**LDLT分解解满秩最小二乘问题** 
 一般无法得到满足对称矩阵A，因此需要使对$A^{T}A$（满足对称）进行分解。将求解问题转换成下面的式子 $AA^{T}X = A^{T}b$
 >定理1： $ A^{T}A$不一定是正定矩阵，但一定是半正定矩阵（即特征值一定大于等于0）
>定理2：$ A^{T}A$是正定矩阵的充分必要条件是 A 满足列满秩

由于r(A)=n，所以$A^{T}A$是对称（正定）矩阵，式子有唯一解:
（1）定义矩阵$C=A^{T}*A$，$d=A^{T}*b$；
（2）对C进行cholesky分解$C=G*G^T$，原式变成$G*G^{T}*x=d$
（3）令$y=G^{T}*x$，原式变成$G*y=d$,求解此方程得到y，然后求解$y=G^{T}*x$得到x。
Cholesky分解要快于$LDL^{T}$分解。

### 5.SVD分解
矩阵的奇异值分解（SVD）在最优化问题、特征值问题、最小二乘问题及广义逆问题中有巨大作用，奇异值分解将QR分解推广到任意的实矩阵，不要求矩阵式可逆，也不要求是方阵。奇异值和特征值相似的重要意义，都是为了提取出矩阵的主要特征。假设A是一个m∗n阶矩阵，如此则存在一个分解m阶正交矩阵U、非负对角阵Σ和n阶正交矩阵V使得：
$A=UΣV^{T}$ 

$A= U\begin{bmatrix}
{\sum}&0\\
0&0\\
\end{bmatrix}V^{T} $

![>](/home/jianing/图片/2022-01-28 00-04-56屏幕截图.png)

Σ对角线上的元素Σi,i即为A的奇异值。而且一般来说，我们会将Σ上的值按从大到小的顺序排列。
通过上面对SVD的简单描述，不难发现，SVD解决了特征值分解中只能针对方阵而没法对更一般矩阵进行分解的问题。所以在实际中，SVD的应用场景比特征值分解更为通用与广泛。将将上面的SVD分解用一个图形象表示如下。

![ruguo](https://img-blog.csdn.net/20180506214904723?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dhbmdzaHVhaWxwcA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
截止到这里为止，很多同学会有疑问了：你这不吃饱了撑得。好好的一个矩阵A，你这为毛要将他表示成三个矩阵。这三个矩阵的规模，一点也不比原来矩阵的规模小好么。而且还要做两次矩阵的乘法。要知道，矩阵乘法可是个复杂度为O(n3)的运算。如果按照之前那种方式分解，肯定是没有任何好处的。矩阵规模大了，还要做乘法运算。关键是奇异值有个牛逼的性质：在大部分情况下，当我们把矩阵Σ里的奇异值按从大到小的顺序呢排列以后，很容易就会发现，奇异值σ减小的速度特别快。在很多时候，前10%甚至前1%的奇异值的和就占了全部奇异值和的99%以上。换句话说，大部分奇异值都很小，基本没什么卵用。。。既然这样，那我们就可以用前面r个奇异值来对这个矩阵做近似。于是，SVD也可以这么写：
$A_{m×n}≈U_{m×r}Σ_{r×r}V_{r×n} $
其中，$r≪m$，$r≪n$。如果用另外一幅图描述这个过程，如下图： 
![这里写图片描述](https://img-blog.csdn.net/20180506215005859?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dhbmdzaHVhaWxwcA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
看了上面这幅图，同学们是不是就恍然大悟：原来的那个大矩阵A，原来可以用右边的那三个小矩阵来表示。当然如果r越大，跟原来的矩阵相似度就越高。如果r=n，那得到的就是原来的矩阵A。但是这样存储与计算的成本就越高。所以，实际在使用SVD的时候，需要我们根据不同的业务场景与需求还有资源情况，合理选择r的大小。本质而言，就是在计算精度与空间时间成本之间做个折中
**SVD分解意义**
按照前面给出的几何含义，SVD 分解可以看成先旋转，然后进行分别缩放，然后再旋转的过程。 ![这里写图片描述](https://img-blog.csdn.net/20180506215220989?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3dhbmdzaHVhaWxwcA==/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)
奇异值分解 (singular value decomposition,SVD) 是另一种正交矩阵分解法；SVD是最可靠的分解法，但是它比QR 分解法要花上近十倍的计算时间。[U,S,V]=svd(A)，其中U和V分别代表两个正交矩阵，而S代表一对角矩阵。 和QR分解法相同， 原矩阵A不必为正方矩阵。使用SVD分解法的用途是解最小平方误差法和数据压缩。

### 5. 示例代码
```c++
#include <iostream>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 100

int main()
{
//方程组形式Ax = b；（假定A是方阵）
 Matrix<double ,MATRIX_SIZE,MATRIX_SIZE> A;
 A = MatrixXd::Random(MATRIX_SIZE,MATRIX_SIZE);

Matrix<double,MATRIX_SIZE,1> b;
b = MatrixXd::Random(MATRIX_SIZE,1);

Matrix<double,MATRIX_SIZE,1> x1;
Matrix<double,MATRIX_SIZE,1> x2;
Matrix<double,MATRIX_SIZE,1> x3;
Matrix<double,MATRIX_SIZE,1> x4;

clock_t  time_stt = clock();
x1 = A.inverse()*b;       //方式1：直接求逆求解x = A的逆 * b//1.直接解法  很可能没有解 仅仅针对方阵才能计算
cout<<"x1 time is:"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
cout<<"x1:"<<x1.transpose()<<endl;

time_stt = clock();
x2 = A.colPivHouseholderQr().solve(b);       //方式2：采用RQ分解
//（在这里，ColPivHouseholderQR是一个QR分解。这里的QR分解的一个很好的折衷方案，因为它适用于所有矩阵，同时速度非常快。）当方程组有解时的出的是真解，若方程组无解得出的是近似解
//eigen提供了很多类型的分解方式
cout<<"x2 time is:"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
cout<<"x2:"<<x2.transpose()<<endl;

time_stt = clock();
x3 = (A.transpose()*A).llt().solve(A.transpose()*b);  //方法3：choleskey分解。
//因为llt分解要求A是对称正定的，一般的矩阵不满足这个条件，故构造新的线性方程：(A的转置*A)*x = （A的转置*b），此方程与原方程同解，同时满足choleskey分解的条件
cout<<"x3 time is:"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
cout<<"x3:"<<x3.transpose()<<endl;

time_stt = clock();
x4 = (A.transpose()*A).ldlt().solve(A.transpose()*b);//方法4：改进的choleskey分解。LDLT分解
cout<<"x4 time is:"<<1000*(clock()-time_stt)/(double)CLOCKS_PER_SEC<<"ms"<<endl;
cout<<"x4:"<<x4.transpose()<<endl;

return 0;

```

## 七. Eigen 几何模块

### 1. 旋转向量：Eigen::AngleAxis
#### 1.1 初始化:
```c++
AngleAxisd rotation_vector(alpha, Vector3d(x, y, z));  //alpha:旋转角度，(x, y, z):旋转轴
AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度,以（0，0，1）为轴旋转了45度
```
#### 1.2 转换为旋转矩阵：

```c++
Matrix3d rotation_matrix;
rotation_matrix = rotation_vector.matrix();//旋转向量转换为旋转矩阵（方式1）
rotation_matrix = rotation_vector.toRotationMatrix();//旋转向量转换为旋转矩阵（方式2）
```
#### 1.3 转换为欧拉角（需要先转换为旋转矩阵）
```c++
euler_angles = rotation_vector.matrix().eulerAngles(2, 1, 0);//// ZYX顺序，即roll pitch yaw顺序
```
#### 1.4 转换为四元数
```c++
Quaternion q = Quaterniond(rotation_vector);
```

### 2. 欧拉角：Eigen::eulerAngles(a0, a1, a2)
用法：a0, a1, a2从前到后依次为旋转轴（下一次旋转使用的旋转轴是上一次旋转之后的），0代表X轴，1代表Y轴，2代表Z轴。
得到的角度值范围：[0, pi], [-pi, pi], [-pi, pi]
#### 可以将旋转矩阵直接转换成欧拉角
```c++
Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即roll pitch yaw顺序
```
等价于：
```c++
rotation_matrix == AngleAxisd(euler_angles[0], Vector3f::UnitZ())
     * AngleAxisd(euler_angles[1], Vector3f::UnitY())
     * AngleAxisd(euler_angles[2], Vector3f::UnitX()); 
```
均代表先绕Z轴旋转euler_angles[0] rad，再绕Y轴旋转euler_angles[1] rad，再绕X轴旋转euler_angles[2] rad。
注意：后一种方式求旋转矩阵时三个旋转向量顺序不能颠倒。

### 3. 欧式变换：Eigen::Isometry
#### 3.1 定义
```c++
 // 欧氏变换矩阵使用 Eigen::Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T.rotate ( rotation_vector );                                     // 按照rotation_vector进行旋转(定义欧式变换的旋转矩阵)
    T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );                     // 把平移向量设成(1,3,4)(定义欧式变换的平移向量)
```

#### 3.2 用变换矩阵进行坐标变换
```c++
 Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t
 cout<<"v tranformed = "<<v_transformed.transpose()<<endl;
```
#### 3.3 转换为4*4变换矩阵
```c++
Matrix4d transform_matrix = T.matrix();
```
### 4. 四元数：Eigen::Quaternion
#### 4.1 初始化
```c++
Quaterniond q(w, x, y, z);//注意：初始化时实部在前，虚部在后，但在内部存储顺序为[x, y, z, w]
```
打印四元数
```c++
cout << q.coeffs() << endl;
```
结果为[x, y, z, w]
#### 4.2 使用四元数旋转向量
```c++
Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
```
####  4.3旋转矩阵赋给它
```c++
    q = Eigen::Quaterniond ( rotation_matrix );
```
#### 4.4 使用四元数旋转向量
```c++
Vector3d v_rotated = q * v;
```
### 5. 参考代码1
```c++
#include <iostream>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <vector>
#include <math.h>
 
using namespace std;
using namespace Eigen;
 
Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
 
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    cout << "Euler2Quaternion result is:" <<endl;
    cout << "x = " << q.x() <<endl;
    cout << "y = " << q.y() <<endl;
    cout << "z = " << q.z() <<endl;
    cout << "w = " << q.w() <<endl<<endl;
    return q;
}
 
Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
 
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    cout << "Quaterniond2Euler result is:" <<endl;
    cout << "x = "<< euler[2] << endl ;
    cout << "y = "<< euler[1] << endl ;
    cout << "z = "<< euler[0] << endl << endl;
}
 
Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;
 
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    cout << "Quaternion2RotationMatrix result is:" <<endl;
    cout << "R = " << endl << R << endl<< endl;
    return R;
}
 
 
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();
    cout << "RotationMatrix2Quaterniond result is:" <<endl;
    cout << "x = " << q.x() <<endl;
    cout << "y = " << q.y() <<endl;
    cout << "z = " << q.z() <<endl;
    cout << "w = " << q.w() <<endl<<endl;
    return q;
}
 
Eigen::Matrix3d euler2RotationMatrix(const double roll, const double pitch, const double yaw)
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
 
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    Eigen::Matrix3d R = q.matrix();
    cout << "Euler2RotationMatrix result is:" <<endl;
    cout << "R = " << endl << R << endl<<endl;
    return R;
}
 
Eigen::Vector3d RotationMatrix2euler(Eigen::Matrix3d R)
{
    Eigen::Matrix3d m;
    m = R;
    Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);
    cout << "RotationMatrix2euler result is:" << endl;
    cout << "x = "<< euler[2] << endl ;
    cout << "y = "<< euler[1] << endl ;
    cout << "z = "<< euler[0] << endl << endl;
    return euler;
}
 
 
int main(int argc, char **argv)
{
 
//this is euler2Quaternion transform function,please input your euler angle//
  euler2Quaternion(0,0,0);
 
//this is Quaternion2Euler transform function,please input your euler angle//
  Quaterniond2Euler(0,0,0,1);
 
//this is Quaternion2RotationMatrix transform function,please input your Quaternion parameter//
  Quaternion2RotationMatrix(0,0,0,1);
 
//this is rotationMatrix2Euler transform function,please input your RotationMatrix parameter like following//
  Eigen::Vector3d x_axiz,y_axiz,z_axiz;
  x_axiz << 1,0,0;
  y_axiz << 0,1,0;
  z_axiz << 0,0,1;
  Eigen::Matrix3d R;
  R << x_axiz,y_axiz,z_axiz;
  rotationMatrix2Quaterniond(R);
 
//this is euler2RotationMatrix transform function,please input your euler angle for the function parameter//
  euler2RotationMatrix(0,0,0);
 
//this is RotationMatrix2euler transform function,please input your euler angle for the function parameter//
  RotationMatrix2euler(R);
 
  cout << "All transform is done!" << endl;
}
```
### 6. 参考代码2
```c++
#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>

/****************************
* 本程序演示了 Eigen 几何模块的使用方法
****************************/

int main ( int argc, char** argv )
{
    // Eigen/Geometry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();//在定义变量时使用Eigen::Matrix4f x =  Eigen::Matrix4f::Identity();即用单位矩阵对x变量进行了初始化
    cout<<rotation_matrix<<endl;
    // 旋转向量使用 AngleAxis, 它底层不直接是Matrix，但运算可以当作矩阵（因为重载了运算符）
    Eigen::AngleAxisd rotation_vector ( M_PI/4, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 45 度,以（0，0，1）为轴旋转了45度
    cout .precision(3);
    cout<<"rotation matrix =\n"<<rotation_vector.matrix() <<endl;                //用matrix()转换成矩阵
    // 也可以直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();
    cout<<rotation_matrix<<endl;
    // 用 AngleAxis 可以进行坐标变换
    Eigen::Vector3d v ( 1,0,0 );
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;
    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles ( 2,1,0 ); // ZYX顺序，即roll pitch yaw顺序
    cout<<"yaw pitch roll = "<<euler_angles.transpose()<<endl;

    // 欧氏变换矩阵使用 Eigen::Isometry
    Eigen::Isometry3d T=Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T.rotate ( rotation_vector );                                     // 按照rotation_vector进行旋转
    T.pretranslate ( Eigen::Vector3d ( 1,3,4 ) );                     // 把平移向量设成(1,3,4)
    cout << "Transform matrix = \n" << T.matrix() <<endl;

    // 用变换矩阵进行坐标变换
    Eigen::Vector3d v_transformed = T*v;                              // 相当于R*v+t
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;

    // 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略

    // 四元数
    // 可以直接把AngleAxis赋值给四元数，反之亦然
    Eigen::Quaterniond q = Eigen::Quaterniond ( rotation_vector );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;   // 请注意coeffs的顺序是(x,y,z,w),w为实部，前三者为虚部
    // 也可以把旋转矩阵赋给它
    q = Eigen::Quaterniond ( rotation_matrix );
    cout<<"quaternion = \n"<<q.coeffs() <<endl;
    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q*v; // 注意数学上是qvq^{-1}
    cout<<"(1,0,0) after rotation = "<<v_rotated.transpose()<<endl;

    return 0;
}
```

 
