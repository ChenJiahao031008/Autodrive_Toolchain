# Autodrive_Toolchain
A Simple Autodrive System Tool Chain, 一个简易的自动驾驶系统工具链

## 1 Dependency

```
gcc >= 5, recommend gcc >= 9.4
cmake >= 3.15.0
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
# 1. download third party libaray
git submodule update --init
# 2. run script to build and test 
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

### 4 API

> Tips：请善用全局搜索相关函数。该章节介绍是以功能为导向而非函数为导向的，因此详细使用可以参考gtest模块或者example模块；

1. 文件读取：
   + **读取TUM数据集中的位姿并存储**：`std::vector<sensorData::PoseData> GetTUMPoseFromFile(std::ifstream &pose_stream, int drop_lines_num)`；
   + **读取文件夹中的所有文件夹或/和文件名，并能够区分是否递归读取子文件**：`void ReadFolder(const std::string &folder, std::vector<std::string> &files, int mode)`；
   + **拷贝/移动文件或者文件夹**：`bool CopyFiles(const std::string &src, const std::string &dst, int mode = 0)`；
   + **读取YAML文件格式的传感器外参**：`LoadExtrinsic(const std::string &file_path, Eigen::Affine3d &extrinsic)`；
   + **读取YAML文件格式的传感器内参**：`bool LoadIntrinsic(const std::string &intrinsics_path, cv::Mat&dist_coeffs, cv::Mat &intrisic_mat)`；
   + **获取文件地址中的文件名，不带扩展名**：`bool GetFileNameInPath(const std::string &path, std::string &filename)`；
   + **获得当前可执行文件目录**：`std::string GetCurrentDir()`；
2. 文件写入：
   + **保存轨迹，支持TUM，KITTI等多种格式**：`void SaveTrajectory(const std::string &filename, const std::vector<sensorData::PoseData> &trajectory, save_options mode = save_options::TUM)`；
3. 数据处理：
   + **多种旋转插值方法**：`PoseSyncData(sensorData::CircularQueue<T> &unsynced_data, sensorData::CircularQueue<T> &synced_data, double sync_time, double max_interval)`；
   + **Eigen，OpenCV内多种数据格式的相互转换**：`class DataConverter`；
4. 数学支持：
   + **梯度下降法接口，使用详见example**：`class GradientDescent`；
   + **线性卡尔曼滤波其接口**：`class KalmanFilter`；
   + **使用M个2d点拟合N阶多项式**：`std::array<double, N + 1> FitPolynomial( const std::array<Eigen::Vector2d, M> &points, double *ptr_error_square)`；
5. 可视化部分
   + **深度图像可视化，可以展示和存储**：`void DepthMapVisualization(onst cv::Mat &depth, display_option option, const std::string name)`；
6. 图像处理部分（包括彩色图像和深度图像）
   + **深度图滤波，采用联合双边滤波算法**：`class JBF`；
   + **深度图补全，采用基于传统方法的[快速深度补全](https://arxiv.org/abs/1802.00036)算法**：`class Kernel`；
   + **基于深度图的显著性检测算法，常用于分离背景**：`cv::Mat DepthSaliencyDetection(const cv::Mat &depth)`
7. python脚本：
   + **根据文本绘制目标检测框**：`scripts/tools/深度学习相关/根据文本绘制目标检测框/plot.py`；
   + **将图像和标签按照相同顺序打乱**：`scripts/tools/深度学习相关/将图像和标签按照相同顺序打乱/shuffle.py`；
   + **遍历文件KITTI格式转换单文件TUM格式**：`scripts/tools/数据处理相关/遍历文件KITTI格式转换单文件TUM格式/convert.py`
   + **标签和图像对齐生成txt文件**：`scripts/tools/数据处理相关/标签和图像对齐生成txt文件/alignment.py`；
   + **读取TXT文件并绘图**：`scripts/tools/数据处理相关/读取TXT文件并绘图/plot.py`；
   + **将图片序列生成视频**：`scripts/tools/数据处理相关/将图片序列生成视频/convert.py`；
   + **TUM格式数据集深度和彩色图像关联：**`scripts/tools/数据处理相关/TUM格式数据集深度和彩色图像关联`；
8. 其他：
   + **不包含ros的简化docker**：`docker/docker_without_ros`；
   + **ros接口示例**：`ros_interface`；

## 5 Todo List

非常欢迎各位一起完善该项目！

**以下是项目原则**：

1. 如非必要，**请不要增加太多依赖库**，保证现有的库与待添加的库功能不冲突。如有了ceres就不需要增加g2o。如果库比较小众，请以第三方库的形式添加；
2. 代码贡献者根据个人需要提交分支，完成所有需求后需要添加单元测试。保证所有单元测试通过后，才进行分支合并，并**删除无用的已有分支**。
3. 请保证**代码风格一致性**，代码标准以**c++17标准**为基准，目前不支持更高版本的标准以保证代码的兼容性；
4. 保证每次提交时**有明确的使用说明**，小功能可以`commit`形式（如修复一些bugs），较大功能的开发需要新增readme文档。
5. **注意代码合法性要求**：在借鉴代码中表明代码来源，如使用Apollo代码需要在相关头文件中指出。不使用严禁开源的代码。

**以下是开发计划**：

- [x] 在math_utils中增加梯度下降、修正牛顿法等无约束优化算法；
- [ ] 内存对齐检查，使其兼容c17以下内存对齐方案；
- [ ] 在math_utils增加qr、lqr、svd分解，随机数生成器；
- [x] 在docker中增加常见的dockerfiles文件，包括一个带有ros的版本和一个不支持ros的版本；
- [ ] 在modules中增加特征提取（点、直线、面）；
- [ ] 在modules中增加点云处理相关操作，如：去畸变、聚类、滤波等；
- [ ] 在example中增加简单优化库的使用，如用最小二乘写一个由3D点拟合平面参数的函数；
- [ ] 在example中对空间栅格进行hash及存储空间栅格的hash容器（简易voxel）；
- [ ] 新增viewer模块，增加可视化实例；
- [x] 代码优化：DataConverter类重复性过高，需要用一种简单的方法降低重复性；

