#pragma once
#include <pcl/filters/voxel_grid.h>

#include "base_filter.hpp"

namespace cloud_processing {

template <typename PointType>
class CloudVoxelFilter : public CloudFilterInterface<PointType> {
 public:
  using CloudType = pcl::PointCloud<PointType>;
  using CloudTypePtr = typename CloudType::Ptr;

  CloudVoxelFilter(){};

  CloudVoxelFilter(const float leaf_size_x, const float leaf_size_y,
                   const float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
  };

  bool SetFilterParam(const float leaf_size_x, const float leaf_size_y,
                      const float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    return true;
  };

  bool Filter(CloudTypePtr& input_cloud_ptr,
              CloudTypePtr& filtered_cloud_ptr) override {
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);
    return true;
  };

 private:
  pcl::VoxelGrid<PointType> voxel_filter_;
};

enum class method : unsigned char { CENTROID = 0, RANDOM = 1 };

template <typename ScalarT, ptrdiff_t EigenDim>
class ContainerVoxelFilter
    : public ContainerFilterInterface<ScalarT, EigenDim> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DynamicType = Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>;
  using VectorType = Eigen::Matrix<ScalarT, Eigen::Dynamic, 1>;

  ContainerVoxelFilter()
      : filter_limit_min_(-FLT_MAX),
        filter_limit_max_(FLT_MAX),
        min_points_per_voxel_(0){};

  ContainerVoxelFilter(ScalarT* leaf_size)
      : filter_limit_min_(-FLT_MAX),
        filter_limit_max_(FLT_MAX),
        min_points_per_voxel_(0) {
    SetFilterParam(leaf_size);
  };

  bool SetFilterParam(ScalarT* leaf_size) {
    leaf_size_ = Eigen::Map<Eigen::Matrix<ScalarT, EigenDim, 1>>(leaf_size);
    return true;
  };

  DynamicType Filter(const Container<ScalarT, EigenDim>& input_data) override {
    DynamicType maxrange = input_data.rowwise().maxCoeff();
    DynamicType minrange = input_data.rowwise().minCoeff();
    DynamicType dim =
        (maxrange - minrange).cwiseQuotient(leaf_size_).array().ceil().matrix();

    DynamicType bottom = minrange.replicate(1, input_data.cols());
    DynamicType leaf = leaf_size_.replicate(1, input_data.cols());
    DynamicType indices =
        (input_data - bottom).cwiseQuotient(leaf).array().floor().matrix();

    std::vector<std::pair<size_t, size_t>> index_list;
    for (size_t i = 0; i < input_data.cols(); ++i) {
      index_list.emplace_back(
          std::make_pair(indices(0, i) + indices(1, i) * dim(0, 0) +
                             indices(2, i) * dim(0, 0) * dim(1, 0),
                         i));
    }
    std::sort(index_list.begin(), index_list.end(),
              [&](std::pair<size_t, size_t>& a, std::pair<size_t, size_t>& b) {
                return a.first < b.first;
              });
    DynamicType sum = DynamicType::Zero(EigenDim, 1);
    size_t point_size = 0;
    size_t type_size = 0;
    DynamicType results(EigenDim, input_data.cols());
    for (size_t i = 0; i < index_list.size() - 1; ++i) {
      size_t id = index_list[i].second;
      std::cout << index_list[i].first << "\n";
      if (index_list[i].first == index_list[i + 1].first) {
        if (choice == method::CENTROID) {
          sum += input_data.col(id);
          point_size++;
        }
      } else {
        if (choice == method::CENTROID) {
          sum += input_data.col(id);
          point_size++;
          results.col(type_size++) = sum / point_size;
          sum = DynamicType::Zero(EigenDim, 1);
          point_size = 0;
        } else {
          results.col(type_size++) = input_data.col(id);
        }
      }
    }
    std::cout << "type_size : " << type_size << std::endl;

    return results.leftCols(type_size);
  };

 private:
  float filter_limit_min_;
  float filter_limit_max_;
  int min_points_per_voxel_;
  Eigen::Matrix<ScalarT, EigenDim, 1> leaf_size_;
  method choice = method::CENTROID;
};

}  // namespace cloud_processing
