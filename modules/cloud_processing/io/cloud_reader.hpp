#pragma once
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>

#include "common/logger.hpp"
#include "interface/DataReader.h"

namespace cloud_processing {

inline void ReadCloudTXTFile(const std::string &path,
                             pcl::PointCloud<pcl::PointNormal>::Ptr &cloud,
                             int drop_line = 0, char delimiter = ' ') {
  std::string line;
  std::ifstream cloud_stream(path.c_str(), std::ios::in);
  if (!cloud_stream.good()) {
    AERROR << "Fail to Open any File, Please Check!";
  }
  for (int i = 0; i < drop_line; i++)
    getline(cloud_stream, line);  // drop this line

  while (getline(cloud_stream, line)) {
    std::stringstream line_ss(line);

    float get_data[6];
    for (size_t i = 0; i < 6; ++i) {
      std::string s;
      getline(line_ss, s, delimiter);
      get_data[i] = std::stof(s);
    }
    pcl::PointNormal pt;
    pt.x = get_data[0];
    pt.y = get_data[1];
    pt.z = get_data[2];
    pt.normal_x = get_data[3];
    pt.normal_y = get_data[4];
    pt.normal_z = get_data[5];
    cloud->points.push_back(pt);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
};

inline void ReadCloudTXTFile(const std::string &path,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                             int drop_line = 0, char delimiter = ' ') {
  std::string line;
  std::ifstream cloud_stream(path.c_str(), std::ios::in);
  if (!cloud_stream.good()) {
    AERROR << "Fail to Open any File, Please Check!";
  }
  for (int i = 0; i < drop_line; i++)
    getline(cloud_stream, line);  // drop this line

  while (getline(cloud_stream, line)) {
    std::stringstream line_ss(line);

    float get_data[3];
    for (size_t i = 0; i < 3; ++i){
      std::string s;
      getline(line_ss, s, delimiter);
      get_data[i] = std::stof(s);
    }
    pcl::PointXYZ pt;
    pt.x = get_data[0];
    pt.y = get_data[1];
    pt.z = get_data[2];
    cloud->points.push_back(pt);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
};

template <typename PointT>
inline void ReadPCDFile(const std::string &path,
                        typename pcl::PointCloud<PointT>::Ptr &cloud) {
  const fs::path _path_(path);
  std::string extension_name = _path_.extension().string();
  if (extension_name != ".pcd" && extension_name != ".PCD") {
    AERROR << "Please Check Extension Name";
    return;
  }
  if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1) {
    PCL_ERROR("Read PCD File Fail!\n");
    return;
  }
};

template <typename PointT>
inline void ReadPLYFile(const std::string &path,
                        typename pcl::PointCloud<PointT>::Ptr &cloud) {
  const fs::path _path_(path);
  std::string extension_name = _path_.extension().string();
  if (extension_name != ".ply" && extension_name != ".PLY") {
    AERROR << "Please Check Extension Name";
    return;
  }
  if (pcl::io::loadPLYFile<PointT>(path, *cloud) == -1) {
    PCL_ERROR("Read PCD File Fail!\n");
    return;
  }
};

}  // namespace cloud_processing
