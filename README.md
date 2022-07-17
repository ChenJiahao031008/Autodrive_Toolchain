# Autodrive_Toolchain
A Simple Autodrive System Tool Chain, 一个简易的自动驾驶系统工具链

## 1 Dependency

```
gcc >= 5, recommend gcc >= 9.4
Eigen >= 3.2.7
boost >= 1.58.0
OpenCV 3.x.x
yaml-cpp 0.7.0
proj4
glog
gtest
```

Besides, if you want to use `ros_interface`,  `ros >= kinetic` must be required. If you want to use `gtest` modules, please set `BUILD_TEST` to  `ON` and `gtest` library must be required.

## 2 How to Use

```bash
bash run.sh build 		   			# 建立工程
bash run.sh build_and_run  			# 建立工程并运行
bash run.sh build_and_test 			# 建立工程并测试
bash run.sh run 		   			# 运行程序
bash run.sh test 		   			# 运行测试
bash run.sh clean 		   			# 清理缓存和build文件
bash run.sh ros_build				# 编译ROS接口模块
bash run.sh ros_run					# 运行预定ros_launch文件
bash run.sh ros_build_and_run		# 编译ROS接口模块并运行预定ros_launch文件
bash run.sh ros_save_traj			# 运行ROS接口下的轨迹保存模块
```

## 3 System Structure

+ common：通用处理模块，依赖最少的库，目前库中包括：日志系统、时间系统、定时器系统；
+ config：放置配置相关文件和参数表，其中config.h配合第三方库easy_config做c++反射使用；
+ docker：放置通用的dockerfile文件；
+ example：一些常用的案例库；
+ gtest：作为gtest单元测试使用，合并分支前需要通过所有的单元测试；
+ interface：常见的接口处理模块，包括数据读入、数据处理、数据输出等；
+ math_utils：常见的数学计算库，包括多项式拟合、卡尔曼滤波器等，作为eigen的补充和封装使用；
+ modules：算法的核心代码；
+ ros_interface：独立模块，提供ROS处理模块；
+ scripts：提供常用的脚本工具；
+ sensor_data：本系统所使用的数据类型和常见数据结构的补充；
+ slam_results：放置数据运行结果数据；
+ third_party：存放第三方库，目前支持backward-cpp（内存追踪库）、easy_config（c++参数反射库）等；

## 4 Todo List

非常欢迎各位一起完善该项目！

**以下是项目原则**：

1. 如非必要，**请不要增加太多依赖库**，保证现有的库与待添加的库功能不冲突。如有了ceres就不需要增加g2o。如果库比较小众，请以第三方库的形式添加；
2. 代码贡献者根据个人需要提交分支，完成所有需求后需要添加单元测试。保证所有单元测试通过后，才进行分支合并，并**删除无用的已有分支**。
3. 请保证**代码风格一致性**，代码标准以**c++17标准**为基准，目前不支持更高版本的标准以保证代码的兼容性；
4. 保证每次提交时**有明确的使用说明**，小功能可以`commit`形式（如修复一些bugs），较大功能的开发需要新增readme文档。
5. **注意代码合法性要求**：在借鉴代码中表明代码来源，如使用Apollo代码需要在相关头文件中指出。不使用严禁开源的代码。

**以下是开发计划**：

- [ ] 在math_utils中增加梯度下降、修正牛顿法等无约束优化算法；
- [ ] 在math_utils增加qr、lqr、svd分解，随机数生成器；
- [ ] 在docker中增加常见的dockerfiles文件，包括一个带有ros的版本和一个不支持ros的版本；
- [ ] 在modules中增加特征提取（点、直线、面）；
- [ ] 在modules中增加点云处理相关操作，如：去畸变、聚类、滤波等；
- [ ] 在example中增加简单优化库的使用，如用最小二乘写一个由3D点拟合平面参数的函数；
- [ ] 在example中对空间栅格进行hash及存储空间栅格的hash容器（简易voxel）；
- [ ] 新增viewer模块，增加可视化实例；
- [ ] 代码优化：DataConverter类重复性过高，需要用一种简单的方法降低重复性；

