#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "common/logger.hpp"
#include "interface/DataReader.h"
#include "modules/cloud_processing/core/container.hpp"
#include "modules/cloud_processing/io/cloud_reader.hpp"
#include "modules/cloud_processing/io/cloud_writer.hpp"
#include "modules/cloud_processing/filter/voxel_filter.hpp"

#ifndef LOG_CORE_DUMP_CAPTURE
#define BACKWARD_HAS_DW 1
#endif

using namespace std;
using namespace Eigen;
using namespace cloud_processing;

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::string path = interface::DataReader::GetCurrentDir() +
                     "/../examples/cloud_processing/testdata/";
  AINFO << path;

  std::string cloud_path = path + "/airplane_0001.txt";
  ReadCloudTXTFile(cloud_path, cloud0_ptr, 0, ',');

  Container<float, 3> container(cloud0_ptr);

  float leaf[3] = {0.05, 0.05, 0.05};
  ContainerVoxelFilter<float, 3> filter(leaf);
  auto results = filter.Filter(container);

  AINFO << results.cols();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  cloud_out1_ptr->resize(results.cols());
  for (size_t i = 0; i < results.cols(); ++i) {
    cloud_out1_ptr->points[i].x = results(0, i);
    cloud_out1_ptr->points[i].y = results(1, i);
    cloud_out1_ptr->points[i].z = results(2, i);
    // cloud_out1_ptr->points[i].x = results[i][0];
    // cloud_out1_ptr->points[i].y = results[i][1];
    // cloud_out1_ptr->points[i].z = results[i][2];
  }
  std::string cloud_out1_path = path + "cloud_filter_out1.pcd";
  SavePCDFile<pcl::PointXYZ>(cloud_out1_path, cloud_out1_ptr, false);
  return 0;
}
