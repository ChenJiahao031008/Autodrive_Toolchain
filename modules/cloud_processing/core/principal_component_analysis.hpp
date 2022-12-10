#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "container.hpp"

namespace cloud_processing {

template <typename ScalarT, ptrdiff_t EigenDim>
class PrincipalComponentAnalysis {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PCAVectorType = Eigen::Matrix<ScalarT, EigenDim, 1>;
  using PCACovType = Eigen::Matrix<ScalarT, EigenDim, EigenDim>;
  using PCADataType = Eigen::Matrix<ScalarT, EigenDim, Eigen::Dynamic>;

 public:
  PrincipalComponentAnalysis(const Container<ScalarT, EigenDim> &data) {
    ComputeMean(data);
    ComputePrinciple();
  }

  ~PrincipalComponentAnalysis() {}

  inline const PCAVectorType &getDataMean() const { return mean_; }

  inline const PCAVectorType &getEigenValues() const { return eigenvalues_; }

  inline const PCACovType &getEigenValuesMatrix() const {
    return (eigenvectors_.inverse() * H * eigenvectors_.transpose().inverse());
  }

  inline const PCACovType &getEigenVectors() const { return eigenvectors_; }

  inline const PCADataType &getAfterMean() const { return x_minus_mean_; }

  inline const double &CheckSVDRes() const {
    PCADataType data_tail =
        eigenvectors_ * getEigenValuesMatrix() * eigenvectors_.transpose();
    return (data_tail - H).norm();
  }

  inline void ComputeMean(const Container<ScalarT, EigenDim> &data) {
    // TODO:利用openmp加速
    mean_ = PCAVectorType::Zero(EigenDim, 1);
    for (size_t i = 0; i < data.cols(); ++i) mean_ += data.col(i);
    mean_ /= data.cols();

    x_minus_mean_ = PCADataType::Zero(EigenDim, data.cols());
    for (size_t i = 0; i < data.cols(); ++i)
      x_minus_mean_.col(i) = data.col(i) - mean_;
  }

  inline Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>
  DimensionReduction(const Container<ScalarT, EigenDim> &data, int dimension) {
    Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> Z =
        eigenvectors_.bottomRows(dimension);
    return Z.transpose() * Z * data;
  };

 protected:
  PCAVectorType mean_;
  PCAVectorType eigenvalues_;
  PCACovType covariance_;
  PCACovType eigenvectors_;
  PCADataType x_minus_mean_;
  PCADataType H;

  inline void ComputePrinciple() {
    H = x_minus_mean_ * x_minus_mean_.transpose();
    Eigen::JacobiSVD<PCADataType> svd(
        H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    eigenvalues_ = svd.singularValues();
    eigenvectors_ = svd.matrixU();
  }
};

using PrincipalComponentAnalysis2f = PrincipalComponentAnalysis<float, 2>;
using PrincipalComponentAnalysis2d = PrincipalComponentAnalysis<double, 2>;
using PrincipalComponentAnalysis3f = PrincipalComponentAnalysis<float, 3>;
using PrincipalComponentAnalysis3d = PrincipalComponentAnalysis<double, 3>;
using PrincipalComponentAnalysisXf =
    PrincipalComponentAnalysis<float, Eigen::Dynamic>;
using PrincipalComponentAnalysisXd =
    PrincipalComponentAnalysis<double, Eigen::Dynamic>;

}  // namespace cloud_processing
