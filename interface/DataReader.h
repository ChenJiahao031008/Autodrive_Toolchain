#pragma once

#include <dirent.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "../common/logger.hpp"
#include "../common/macros.hpp"
#include "../sensor_data/pose_data.hpp"

namespace interface {

class DataReader {
 public:
  // 读取TUM格式的位姿数据，drop数值表示忽略前多少行
  static std::vector<sensorData::PoseData> GetTUMPoseFromFile(
      std::ifstream &pose_stream, int drop_lines_num = 0);

  // 读取文件夹中的所有文件夹和文件名
  // mode=0:不允许递归, 输出文件和文件夹; mode=1:允许递归, 但是只输出文件名;
  // mode=2:允许递归, 但是只输出文件夹; mode=3: 允许递归, 输出文件名和文件路径;
  static void ReadFolder(const std::string &folder,
                         std::vector<std::string> &files, int mode = 3);

  // (推荐)拷贝文件或者文件夹: 如果源为普通文件, 目标为文件夹形式,
  // 则会自动创建同名普通文件, 如果目标为普通文件, 则会覆盖目标文件.
  // 模式0表示拷贝不改变源文件, 模式1表示移动文件
  static bool CopyFiles(const std::string &src, const std::string &dst,
                        int mode = 0);

  // 拷贝文件夹, 支持递归拷贝
  static bool CopyDirectory(const std::string &strSourceDir,
                            const std::string &strDestDir);

  // 读取YAML文件格式的传感器外参
  static void LoadExtrinsic(const std::string &file_path,
                            Eigen::Affine3d &extrinsic);

  // 读取YAML文件格式的传感器内参
  static bool LoadIntrinsic(const std::string &intrinsics_path,
                            cv::Mat &dist_coeffs, cv::Mat &intrisic_mat);

  // 获取文件地址中的文件名
  static bool GetFileNameInPath(const std::string &path, std::string &filename);

  // 获得文件地址中的扩展名
  static std::string GetFileExtension(const std::string &path);

  // 获取程序的当前目录
  static std::string GetCurrentDir();

 private:
  DECLARE_SINGLETON(DataReader)
};

}  // namespace interface
