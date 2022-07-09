#pragma once

#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "matrix_operations.hpp"
#include "../common/logger.hpp"

namespace math_utils {


template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
class KalmanFilter {

private:
    bool is_initialized_ = false;
    /**
     *  state transition: x = F * x + B * u + w, where w ~ N(0, Q); x ~ N(0, P)
     *  measurement: z = H * x + v, where v ~ N(0, R);
     */
    Eigen::Matrix<T, XN, 1> x_;  // current state
    Eigen::Matrix<T, ZN, 1> y_;  // prediction bias, y = z - H * x

    Eigen::Matrix<T, XN, XN> P_; // curent state belief covariance
    Eigen::Matrix<T, XN, ZN> K_; // Kalman gain;

    Eigen::Matrix<T, XN, XN> F_; // state transition matrix
    Eigen::Matrix<T, XN, XN> Q_; // covariance of the state transition noise
    Eigen::Matrix<T, XN, UN> B_; // control matrix in state transition rule

    Eigen::Matrix<T, ZN, XN> H_; // observation matrix
    Eigen::Matrix<T, ZN, ZN> R_; // covariance of observation noise
    Eigen::Matrix<T, ZN, ZN> S_; // observation covariance;

public:
    KalmanFilter()
    {
        F_.setIdentity();
        Q_.setZero();
        H_.setIdentity();
        R_.setZero();
        B_.setZero();
        x_.setZero();
        P_.setZero();
        y_.setZero();
        S_.setZero();
        K_.setZero();
    }

    KalmanFilter(const Eigen::Matrix<T, XN, 1> &x,
                 const Eigen::Matrix<T, XN, XN> &P)
        : KalmanFilter() { SetStateEstimate(x, P); }

    void SetStateEstimate(const Eigen::Matrix<T, XN, 1> &x,
                          const Eigen::Matrix<T, XN, XN> &P)
    {
        x_ = x;
        P_ = P;
        is_initialized_ = true;
    }

    virtual ~KalmanFilter() {}

    /**
     * @brief Predict: x_k = F_k * x_k-1 + B_k * u_k
     *                 P_k = F_k * P_k-1 * F_k' + Q_k
     */
    void Predict(
        const Eigen::Matrix<T, UN, 1> &u = Eigen::Matrix<T, UN, 1>::Zero());
    /**
     * @brief Correct: K = P_k * H_k' * (H_k * P_k * H_k' + R_k )^{-1}
     *                 x_k = x_k + K * (z - H_k * x_k)
     *                 P_k = (I - K * H_k) * P_k
     */
    void Correct(const Eigen::Matrix<T, ZN, 1> &z);

    bool IsInitialized() const { return is_initialized_; }

    void SetTransitionMatrix(const Eigen::Matrix<T, XN, XN> &F) { F_ = F; }
    void SetTransitionNoise(const Eigen::Matrix<T, XN, XN> &Q) { Q_ = Q; }
    void SetObservationMatrix(const Eigen::Matrix<T, ZN, XN> &H) { H_ = H; }
    void SetObservationNoise(const Eigen::Matrix<T, ZN, ZN> &R) { R_ = R; }
    void SetStateCovariance(const Eigen::Matrix<T, XN, XN> &P) { P_ = P; }
    void SetControlMatrix(const Eigen::Matrix<T, XN, UN> &B) { B_ = B; }

    const Eigen::Matrix<T, XN, XN> &GetTransitionMatrix() const { return F_; }
    const Eigen::Matrix<T, XN, XN> &GetTransitionNoise() const { return Q_; }
    const Eigen::Matrix<T, ZN, XN> &GetObservationMatrix() const { return H_; }
    const Eigen::Matrix<T, ZN, ZN> &GetObservationNoise() const { return R_; }
    const Eigen::Matrix<T, XN, UN> &GetControlMatrix() const { return B_; }

    Eigen::Matrix<T, XN, 1> GetStateEstimate() const { return x_; }
    Eigen::Matrix<T, XN, XN> GetStateCovariance() const { return P_; }

    std::string DebugString() const;

};

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::Predict(
    const Eigen::Matrix<T, UN, 1> &u) {
  ACHECK(is_initialized_);

  x_ = F_ * x_ + B_ * u;

  P_ = F_ * P_ * F_.transpose() + Q_;
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline void KalmanFilter<T, XN, ZN, UN>::Correct(
    const Eigen::Matrix<T, ZN, 1> &z) {
  ACHECK(is_initialized_);
  y_ = z - H_ * x_;

  S_ = static_cast<Eigen::Matrix<T, ZN, ZN>>(H_ * P_ * H_.transpose() + R_);

  K_ = static_cast<Eigen::Matrix<T, XN, ZN>>(P_ * H_.transpose() *
                                             PseudoInverse<T, ZN>(S_));

  x_ = x_ + K_ * y_;

  P_ = static_cast<Eigen::Matrix<T, XN, XN>>(
      (Eigen::Matrix<T, XN, XN>::Identity() - K_ * H_) * P_);
}

template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
inline std::string KalmanFilter<T, XN, ZN, UN>::DebugString() const {
  Eigen::IOFormat clean_fmt(4, 0, ", ", " ", "[", "]");
  std::stringstream ss;
  ss << "F = " << F_.format(clean_fmt) << "\n"
     << "B = " << B_.format(clean_fmt) << "\n"
     << "H = " << H_.format(clean_fmt) << "\n"
     << "Q = " << Q_.format(clean_fmt) << "\n"
     << "R = " << R_.format(clean_fmt) << "\n"
     << "x = " << x_.format(clean_fmt) << "\n"
     << "P = " << P_.format(clean_fmt) << "\n";
  return ss.str();
}

}  // namespace math_utils

