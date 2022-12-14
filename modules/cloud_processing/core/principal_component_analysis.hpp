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

  using DynamicType = Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>;
  using VectorType = Eigen::Matrix<ScalarT, Eigen::Dynamic, 1>;

 public:
  PrincipalComponentAnalysis(const Container<ScalarT, EigenDim> &data,
                             bool transpose = false)
      : transpose_(transpose) {
    ComputeMean(data);
    Decomposition();
  }

  ~PrincipalComponentAnalysis() {}

  inline const VectorType &getDataMean() const { return mean_; }
  inline const VectorType &getEigenValues() const { return eigenvalues_; }
  inline const DynamicType &getEigenValuesMatrix() const {
    return (eigenvectors_.inverse() * H_ * eigenvectors_.transpose().inverse());
  }
  inline const DynamicType &getEigenVectors() const { return eigenvectors_; }
  inline const DynamicType &getAfterMean() const { return x_minus_mean_; }
  inline const double &CheckSVDRes() const {
    DynamicType data_tail =
        eigenvectors_ * getEigenValuesMatrix() * eigenvectors_.transpose();
    return (data_tail - H_).norm();
  }

  DynamicType DimensionReduction(const Container<ScalarT, EigenDim> &data,
                                 int dimension) {
    DynamicType Z = eigenvectors_.bottomRows(dimension);
    if (!transpose_)
      return Z.transpose() * Z * data;
    else
      return Z.transpose() * Z * data.transpose();
  };

 private:
  VectorType mean_;
  VectorType eigenvalues_;
  DynamicType eigenvectors_;
  DynamicType x_minus_mean_;
  DynamicType H_;
  bool transpose_;

  void ComputeMean(const Container<ScalarT, EigenDim> &data) {
    // TODO:利用openmp加速
    if (!transpose_) {
      mean_ = VectorType::Zero(data.rows(), 1);
      for (size_t i = 0; i < data.cols(); ++i) mean_ += data.col(i);
      mean_ /= data.cols();

      x_minus_mean_ = DynamicType::Zero(data.rows(), data.cols());
      for (size_t i = 0; i < data.cols(); ++i)
        x_minus_mean_.col(i) = data.col(i) - mean_;
    } else {
      mean_ = VectorType::Zero(data.cols(), 1);
      for (size_t i = 0; i < data.rows(); ++i) mean_ += data.row(i).transpose();
      mean_ /= data.rows();

      x_minus_mean_ = DynamicType::Zero(data.cols(), data.rows());
      for (size_t i = 0; i < data.rows(); ++i)
        x_minus_mean_.col(i) = data.row(i).transpose() - mean_;
    }
  }

  void Decomposition() {
    H_ = x_minus_mean_ * x_minus_mean_.transpose();
    Eigen::JacobiSVD<DynamicType> svd(
        H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
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

enum class kernel_type : unsigned char {
  Linear = 0,
  Polynomial = 1,
  Gaussian = 2,
  Laplacian = 3
};

template <typename ScalarT, ptrdiff_t EigenDim>
class KernelPrincipalComponentAnalysis {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using DynamicType = Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>;
  using VectorType = Eigen::Matrix<ScalarT, Eigen::Dynamic, 1>;

  KernelPrincipalComponentAnalysis(const Container<ScalarT, EigenDim> &data,
                                   kernel_type kp = kernel_type::Gaussian)
      : kernel(kp), gamma(0.001), constant(1.0), order(2.0) {
    ComputeGramMatrix(data);
  };

  void ComputeGramMatrix(const Container<ScalarT, EigenDim> &data) {
    size_t N = data.rows();
    K = DynamicType::Zero(N, N);
    for (size_t i = 0; i < N; ++i) {
      for (size_t j = i; j < N; ++j) {
        K(i, j) = K(j, i) = Kernel(data.row(i), data.row(j));
      }
    }
    DynamicType J = DynamicType::Ones(N, N);
    K = K - 1.0f / N * (J * K + K * J) + 1.0f / (N * N) * J * K * J;
  };

  DynamicType Decomposition(const Container<ScalarT, EigenDim> &data,
                            int dimension) {
    Eigen::JacobiSVD<DynamicType> svd(
        K, Eigen::ComputeFullU | Eigen::ComputeFullV);
    DynamicType eigenvalues = svd.singularValues();
    DynamicType eigenvectors = svd.matrixU();
    // todo: 加断言保证输入维度范围
    // todo: eigenval > 0?
    size_t N = eigenvectors.rows();
    DynamicType Z(dimension, N);
    for (size_t i = 0; i < dimension; ++i) {
      DynamicType alpha = eigenvectors.row(N - i);
      alpha.normalize();
      alpha /= std::sqrt(eigenvalues(N - i, 0) * eigenvalues(N - i, 0));
      Z.row(i) = alpha;
    }

    DynamicType result(N, data.cols());
    for (size_t i = 0; i < N; i++) {
      for (size_t j = 0; j < data.cols(); j++) {
        for (size_t k = 0; k < dimension; k++) {
          result(i, j) += K(i, k) * Z(k, j);
        }
      }
    }
    return result;
  };

 private:
  DynamicType K;
  kernel_type kernel;
  ScalarT gamma;
  ScalarT order;
  ScalarT constant;

  ScalarT Kernel(const VectorType &a, const VectorType &b) {
    if (kernel == kernel_type::Linear)
      return a.dot(b);
    else if (kernel == kernel_type::Polynomial)
      return (std::pow(a.dot(b) + constant, order));
    else if (kernel == kernel_type::Gaussian)
      return (std::exp(-gamma * ((a - b).squaredNorm())));
    else if (kernel == kernel_type::Laplacian)
      return (std::exp(-gamma * ((a - b).template lpNorm<1>())));
  }
};

}  // namespace cloud_processing
