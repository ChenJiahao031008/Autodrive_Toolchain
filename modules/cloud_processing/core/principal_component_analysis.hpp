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
    // TODO:利用openmp加速, 试着选不同轴处理
    mean_ = PCAVectorType::Zero(data.rows(), 1);
    for (size_t i = 0; i < data.cols(); ++i) mean_ += data.col(i);
    mean_ /= data.cols();

    x_minus_mean_ = PCADataType::Zero(data.rows(), data.cols());
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

template <typename ScalarT, ptrdiff_t EigenDim>
class KernelPrincipalComponentAnalysis {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum class kernel_type : unsigned char {
    Linear = 0,
    Polynomial = 1,
    Gaussian = 2,
    Laplacian = 3
  };
  using DynamicType = Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>;

  KernelPrincipalComponentAnalysis(const Container<ScalarT, EigenDim> &data) {
    ComputeGramMatrix(data);
  };

  void ComputeGramMatrix(const Container<ScalarT, EigenDim> &data,
                         kernel_type kernel = kernel_type::Linear) {
    size_t N = data.rows();
    K = DynamicType::Zero(N, N);
    for (size_t i = 0; i < N; ++i) {
      for (size_t j = 0; j < N; ++j) {
        if (kernel == kernel_type::Linear) {
          K(i, j) = data.row(i) * data.row(j).transpose();
        } else if (kernel == kernel_type::Polynomial) {
          K(i, j) = std::pow(1 + data.row(i) * data.row(j).transpose(), param);
        } else if (kernel == kernel_type::Gaussian) {
          DynamicType diff = (data.row(i) - data.row(j)).transpose();
          K(i, j) = std::exp(-param * diff.squaredNorm());
        } else if (kernel == kernel_type::Laplacian) {
          DynamicType diff = (data.row(i) - data.row(j)).transpose();
          K(i, j) = std::exp(-param * diff.template lpNorm<1>());
        }
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
      alpha /= std::sqrt(eigenvalues(N - i, 0));
      Z.row(i) = alpha;
    }
    std::cout << "N: " << N << "\n";
    std::cout << "Z.rows(): " << Z.rows() << "\n";
    std::cout << "Z.cols(): " << Z.cols() << "\n";
    std::cout << "K.rows(): " << K.rows() << "\n";
    std::cout << "K.cols(): " << K.cols() << "\n";
    std::cout << "data.rows(): " << data.rows() << "\n";
    std::cout << "data.cols(): " << data.cols() << "\n";
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
  ScalarT param = 15;
};

}  // namespace cloud_processing
