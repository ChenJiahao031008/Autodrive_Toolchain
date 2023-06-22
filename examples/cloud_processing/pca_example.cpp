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
#include "modules/cloud_processing/io/cloud_writer.hpp"

#ifndef LOG_CORE_DUMP_CAPTURE
#define BACKWARD_HAS_DW 1
#endif

using namespace std;
using namespace Eigen;
using namespace cloud_processing;

int main(int argc, char **argv) {
  // // test
  // MatrixXf M1 = MatrixXf::Random(3, 8);
  // cout << "Column major input:" << endl << M1 << "\n";
  // cout << "M1.outerStride() = " << M1.outerStride() << endl;
  // Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Aligned,
  //     OuterStride<>>
  //     M2(M1.data(), M1.rows() - 1, M1.cols(), OuterStride<>(M1.rows()));
  // cout << "1 column over 3:" << endl << M2 << "\n";
  // cout << "M2 size:" << endl << M2.rows() << " " << M2.cols() << "\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::string path = interface::DataReader::GetCurrentDir() +
                     "/../examples/cloud_processing/testdata/";
  AINFO << path;

  std::string cloud_path = path + "/airplane_0001.txt";
  ReadCloudTXTFile(cloud_path, cloud0_ptr, 0, ',');

  Eigen::MatrixXf points(cloud0_ptr->size(), 3);
  for (size_t i = 0; i < cloud0_ptr->size(); ++i) {
    points.row(i) = Eigen::Vector3f(cloud0_ptr->points[i].x, cloud0_ptr->points[i].y,
                        cloud0_ptr->points[i].z).transpose();
  }
  Eigen::MatrixXf points_transpose = points.transpose();

  Container<float, 3> container(cloud0_ptr);
  PrincipalComponentAnalysis<float, 3> pca(container, true);
  AINFO << pca.getEigenVectors().rows() << " " << pca.getEigenVectors().cols();
  Eigen::MatrixXf tmp1 = pca.DimensionReduction(container, 1000);

  Container<float, 3> container2(cloud0_ptr);
  PrincipalComponentAnalysis<float, 3> pca2(cloud0_ptr);
  AINFO << pca2.getEigenVectors().rows() << " " << pca2.getEigenVectors().cols();
  AINFO << container2.rows() << " " << container2.cols();
  Eigen::MatrixXf tmp2 = pca2.DimensionReduction(container2, 2);

  // fix bugs: -1 maybe not right
  Container<float, 10000> container3(points);
  KernelPrincipalComponentAnalysis<float, 10000> kpca(container3);
  Eigen::MatrixXf tmp3 = kpca.Decomposition(container3, 10000);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out1_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out2_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out3_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  cloud_out1_ptr->resize(cloud0_ptr->size());
  cloud_out2_ptr->resize(cloud0_ptr->size());
  cloud_out3_ptr->resize(cloud0_ptr->size());
  std::cout << "tmp1 : " << tmp1.rows() << " " << tmp1.cols() << std::endl;
  std::cout << "tmp2 : " << tmp2.rows() << " " << tmp2.cols() << std::endl;
  std::cout << "tmp3 : " << tmp3.rows() << " " << tmp3.cols() << std::endl;
  for (size_t i = 0; i < cloud0_ptr->size(); ++i) {
    cloud_out1_ptr->points[i].x = tmp1(i, 0);
    cloud_out1_ptr->points[i].y = tmp1(i, 1);
    cloud_out1_ptr->points[i].z = tmp1(i, 2);
    cloud_out2_ptr->points[i].x = tmp2(0, i);
    cloud_out2_ptr->points[i].y = tmp2(1, i);
    cloud_out2_ptr->points[i].z = tmp2(2, i);
    cloud_out3_ptr->points[i].x = tmp3(i, 0);
    cloud_out3_ptr->points[i].y = tmp3(i, 1);
    cloud_out3_ptr->points[i].z = tmp3(i, 2);
  }
  std::string cloud_input_path = path + "cloud_pca_input.pcd";
  std::string cloud_out1_path = path + "cloud_pca_out1.pcd";
  std::string cloud_out2_path = path + "cloud_pca_out2.pcd";
  std::string cloud_out3_path = path + "cloud_pca_out3.pcd";
  SavePCDFile<pcl::PointXYZ>(cloud_input_path, cloud0_ptr, false);
  SavePCDFile<pcl::PointXYZ>(cloud_out1_path, cloud_out1_ptr, false);
  SavePCDFile<pcl::PointXYZ>(cloud_out2_path, cloud_out2_ptr, false);
  SavePCDFile<pcl::PointXYZ>(cloud_out3_path, cloud_out3_ptr, false);

  return 0;
}
