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

template <typename PointT>
inline void SavePCDFile(const std::string &path,
                        typename pcl::PointCloud<PointT>::Ptr &cloud,
                        bool binary = true) {
  const fs::path _path_(path);
  std::string extension_name = _path_.extension().string();
  if (extension_name != ".pcd" && extension_name != ".PCD") {
    AERROR << "Please Check Extension Name";
    return;
  }
  if (!binary)
    pcl::io::savePCDFileASCII(path, *cloud);
  else
    pcl::io::savePCDFileBinary(path, *cloud);
};

template <typename PointT>
inline void SavePLYFile(const std::string &path,
                        typename pcl::PointCloud<PointT>::Ptr &cloud,
                        bool binary = true) {
  const fs::path _path_(path);
  std::string extension_name = _path_.extension().string();
  if (extension_name != ".ply" && extension_name != ".PLY") {
    AERROR << "Please Check Extension Name";
    return;
  }
  pcl::PLYWriter writer;
  writer.write(path, *cloud, binary);
};

}  // namespace cloud_processing
