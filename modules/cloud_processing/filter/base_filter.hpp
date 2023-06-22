#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#define FLT_MAX 3.402823466e+38F
#define FLT_MIN 1.175494351e-38F

namespace cloud_processing {

template <typename PointType>
class CloudFilterInterface {
 public:
  using CloudType = pcl::PointCloud<PointType>;
  using CloudTypePtr = typename CloudType::Ptr;

  virtual ~CloudFilterInterface() = default;

  virtual bool Filter(CloudTypePtr& input_cloud_ptr,
                      CloudTypePtr& filtered_cloud_ptr) = 0;
};

template <typename ScalarT, ptrdiff_t EigenDim>
class ContainerFilterInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DynamicType = Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>;
  using VectorType = Eigen::Matrix<ScalarT, Eigen::Dynamic, 1>;

  virtual ~ContainerFilterInterface() = default;

  virtual DynamicType Filter(
      const Container<ScalarT, EigenDim>& input_data) = 0;
};
;

}  // namespace cloud_processing
