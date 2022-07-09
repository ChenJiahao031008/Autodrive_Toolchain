#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/SVD"

#include "../common/logger.hpp"

namespace math_utils {

/**
 * @brief
 * 功能：使用矩阵分解的方法计算矩阵的伪逆，将所有的特征值的绝对值小于epsilon的值设置为0.
 *
 * @param m The square matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 *
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int N>
Eigen::Matrix<T, N, N> PseudoInverse(const Eigen::Matrix<T, N, N> &m,
                                     const double epsilon = 1.0e-6) {
  Eigen::JacobiSVD<Eigen::Matrix<T, N, N>> svd(
      m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return static_cast<Eigen::Matrix<T, N, N>>(
      svd.matrixV() *
      (svd.singularValues().array().abs() > epsilon)
          .select(svd.singularValues().array().inverse(), 0)
          .matrix()
          .asDiagonal() *
      svd.matrixU().adjoint());
}

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a given matrix,
 * rounding all eigenvalues with absolute value bounded by epsilon to zero.
 *
 * @param m The matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 *
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int M, unsigned int N>
Eigen::Matrix<T, N, M> PseudoInverse(const Eigen::Matrix<T, M, N> &m,
                                     const double epsilon = 1.0e-6) {
  Eigen::Matrix<T, M, M> t = m * m.transpose();
  return static_cast<Eigen::Matrix<T, N, M>>(m.transpose() *
                                             PseudoInverse<T, M>(t));
}

}  // namespace math_utils

