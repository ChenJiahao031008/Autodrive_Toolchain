#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <dirent.h>

#include <boost/algorithm/string.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "../sensor_data/pose_data.hpp"
#include "../common/logger.hpp"

namespace interface
{
// 读取TUM格式的位姿数据
std::vector<sensorData::PoseData> GetTUMPoseFromFile(std::ifstream &pose_stream, int drop_lines_num = 0);

// 读取文件夹中的所有文件名
std::vector<std::string> ReadFolder(const std::string &folder);

// (推荐)拷贝文件或者文件夹: 如果源为普通文件，目标为文件夹形式，则会自动创建同名普通文件，如果目标为普通文件，则会覆盖目标文件. 模式0表示拷贝不改变源文件， 模式1表示移动文件
bool CopyFiles(const std::string &src, const std::string &dst, int mode = 0);

// 拷贝文件夹，支持递归拷贝
bool CopyDirectory(const std::string &strSourceDir, const std::string &strDestDir);

// 读取YAML文件格式的传感器外参
void LoadExtrinsic(const std::string &file_path, Eigen::Affine3d &extrinsic);

// 读取YAML文件格式的传感器内参
bool LoadIntrinsic(const std::string &intrinsics_path, cv::Mat *dist_coeffs, cv::Mat *intrisic_mat);

// 获取文件地址中的文件名
bool GetFileNameInPath(const std::string &path, std::string &filename);

} // namespace interface


