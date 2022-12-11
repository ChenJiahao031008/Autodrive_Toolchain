#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <type_traits>

#include "common/logger.hpp"

namespace cloud_processing {

template <typename PType, ptrdiff_t EigenDim>
class Container
    : public Eigen::Map<Eigen::Matrix<PType, EigenDim, Eigen::Dynamic>,
                        Eigen::Aligned, Eigen::OuterStride<>> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  using EigenT = Eigen::Matrix<PType, EigenDim, 1>;
  using EigenMatrixT = Eigen::Matrix<PType, EigenDim, Eigen::Dynamic>;
  using ContainerT =
      Eigen::Map<EigenMatrixT, Eigen::Aligned, Eigen::OuterStride<>>;

  // Eigen::Map 相关
  Container(const ContainerT &data)
      : ContainerT((PType *)data.data(), data.rows(), data.cols(),
                   Eigen::OuterStride<>(data.cols())){};

  // Eigen::Matrix相关
  Container(const Eigen::Matrix<PType, Eigen::Dynamic, Eigen::Dynamic> &data)
      : ContainerT((PType *)data.data(), data.rows(), data.cols(),
                   Eigen::OuterStride<>(data.rows())){};

  // std::vector<PType> 相关
  template <ptrdiff_t Dim = EigenDim,
            class = typename std::enable_if<Dim != Eigen::Dynamic>::type>
  Container(std::vector<PType> &data)
      : ContainerT((PType *)data.data(), EigenDim, data.size() / EigenDim,
                   Eigen::OuterStride<>(EigenDim)){};

  template <ptrdiff_t Dim = EigenDim,
            class = typename std::enable_if<Dim == Eigen::Dynamic>::type>
  Container(std::vector<PType> &data, size_t dim)
      : ContainerT((PType *)data.data(), dim, data.size() / dim){};

  // std::vector<EigenT> 相关
  template <ptrdiff_t Dim = EigenDim,
            class = typename std::enable_if<
                Dim != Eigen::Dynamic &&
                sizeof(Eigen::Matrix<PType, Dim, 1>) % 16 != 0>::type>
  Container(std::vector<EigenT> &data)
      : ContainerT((PType *)data.data(), EigenDim, data.size(),
                   Eigen::OuterStride<>(EigenDim)) {
    std::cout << "Alignment Recommended.\n";
  };

  template <ptrdiff_t Dim = EigenDim,
            class = typename std::enable_if<Dim != Eigen::Dynamic>::type>
  Container(std::vector<EigenT, Eigen::aligned_allocator<EigenT>> &data)
      : ContainerT((PType *)data.data(), EigenDim, data.size(),
                   Eigen::OuterStride<>(EigenDim)){};

  // pcl::PointCloud 相关
  Container(pcl::PointCloud<pcl::PointXYZ>::Ptr &data)
      : ContainerT(reinterpret_cast<float *>(&(data->points[0])), EigenDim,
                   data->size(), Eigen::OuterStride<>(4)){};
};

using Container2f = Container<float, 2>;
using Container2d = Container<double, 2>;
using Container3f = Container<float, 3>;
using Container3d = Container<double, 3>;
using Container4f = Container<float, 4>;
using Container4d = Container<double, 4>;
using ContainerXf = Container<float, Eigen::Dynamic>;
using ConstDataMatrixMapXd = Container<double, Eigen::Dynamic>;

}  // namespace cloud_processing
