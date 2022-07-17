# Autodrive_Toolchain
A Simple Autodrive System Tool Chain, 一个简易的自动驾驶系统工具链

## 1 Dependency

```
gcc >= 9
Eigen 3.2.7
glog
gtest
boost
opencv test in 3.3.1/3.4.4

```

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

