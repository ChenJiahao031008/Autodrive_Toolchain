#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "common/logger.hpp"
#include "interface/DataReader.h"
#include "modules/cloud_processing/core/container.hpp"
#include "modules/cloud_processing/core/principal_component_analysis.hpp"
#include "modules/cloud_processing/io/cloud_reader.hpp"
// #include "modules/cloud_processing/io/cloud_writer.hpp"

#ifndef LOG_CORE_DUMP_CAPTURE
#define BACKWARD_HAS_DW 1
#endif

using namespace std;
using namespace Eigen;
using namespace cloud_processing;

int main(int argc, char **argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::string path = interface::DataReader::GetCurrentDir() +
                     "/../examples/cloud_processing/testdata/";
  AINFO << path;

  // std::string cloud_path = path + "cloud_100.txt";
  // ReadPCDFile<pcl::PointXYZ>(cloud_path, cloud0_ptr);
  // for (size_t i = 0; i < cloud0_ptr->size(); ++i) {
  //   cloud0_ptr->points[i].x /= 1000.0f;
  //   cloud0_ptr->points[i].y /= 1000.0f;
  //   cloud0_ptr->points[i].z /= 1000.0f;
  // }

  std::string cloud_path = path + "/desk_0001.txt";
  ReadCloudTXTFile(cloud_path, cloud0_ptr, 0, ',');
  for (size_t i = 0; i < 5; ++i) {
    AINFO << cloud0_ptr->points[i].x << " " << cloud0_ptr->points[i].y << " "
          << cloud0_ptr->points[i].z;
  }

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      points;
  points.emplace_back(0, 0, 0);
  points.emplace_back(1, 0, 0);
  points.emplace_back(0, 100, 0);
  points.emplace_back(0, 0, 1000);
  points.emplace_back(0, 100, 1000);
  points.emplace_back(1, 0, 1000);
  points.emplace_back(1, 100, 0);
  points.emplace_back(1, 100, 1000);
  Container<float, 3> container(points);
  // for (size_t i = 0; i < points.size(); ++i) {
  //   AINFO << container.col(i).transpose();
  // }

  Container<float, 3> container2(cloud0_ptr);
  // AINFO << container2.cols();
  // AINFO << container2.rows();  // 3
  // AINFO << container2.col(2).transpose();

  // MatrixXf M1 = MatrixXf::Random(3, 8);
  // cout << "Column major input:" << endl << M1 << "\n";
  // cout << "M1.outerStride() = " << M1.outerStride() << endl;
  // cout << "M1.innerStride() = " << M1.innerStride() << endl;
  // Map<Eigen::Matrix<float, 2, Eigen::Dynamic>, Eigen::Aligned, OuterStride<>>
  //     M2(M1.data(), M1.rows() - 1, M1.cols(), OuterStride<>(M1.rows()));
  // cout << "1 column over 3:" << endl << M2 << "\n";

  PrincipalComponentAnalysis<float, 3> pca(container2);
  AINFO << pca.getDataMean().transpose();
  AINFO << pca.getEigenValues().transpose();
  AINFO << pca.getEigenVectors().rows() << " " << pca.getEigenVectors().cols();
  Eigen::MatrixXf tmp = pca.DimensionReduction(container2, 2);
  AINFO << tmp.rows() << " " << tmp.cols();
  cloud_out_ptr->resize(cloud0_ptr->size());
  for (size_t i = 0; i < cloud0_ptr->size(); ++i) {
    cloud_out_ptr->points[i].x = tmp(0, i);
    cloud_out_ptr->points[i].y = tmp(1, i);
    cloud_out_ptr->points[i].z = tmp(2, i);
  }
  // std::string cloud_out_ply_path = path + "cloud_pca_out_.ply";
  // std::string cloud_out_pcd_path = path + "cloud_pca_out.pcd";
  // SavePLYFile<pcl::PointXYZ>(cloud_out_ply_path, cloud_out_ptr);
  // SavePCDFile<pcl::PointXYZ>(cloud_out_pcd_path, cloud_out_ptr);

  return 0;
}
