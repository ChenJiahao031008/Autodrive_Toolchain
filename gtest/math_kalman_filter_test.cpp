#include <gtest/gtest.h>
#include "../common/logger.hpp"
#include "../math_utils/kalman_filter.hpp"

using namespace math_utils;
class KalmanFilterTest : public ::testing::Test
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    KalmanFilterTest() : kf_() {}

    virtual void SetUp()
    {
        // Initial state
        Eigen::Matrix<double, 2, 1> x;
        x(0, 0) = 0.0;
        x(1, 0) = 1.0;

        // Initial state belief covariance
        Eigen::Matrix<double, 2, 2> P;
        P.setZero();
        P(0, 0) = 0.1;
        P(1, 1) = 0.1;

        // Transition matrix
        Eigen::Matrix<double, 2, 2> F;
        F.setZero();
        F(0, 0) = 1.0;
        F(0, 1) = 1.0;
        F(1, 1) = 1.0;

        // Transition noise covariance
        Eigen::Matrix<double, 2, 2> Q;
        Q.setZero();
        Q(0, 0) = 0.01;
        Q(1, 1) = 0.01;

        // Observation matrix
        Eigen::Matrix<double, 1, 2> H;
        H.setIdentity();

        // Observation noise covariance
        Eigen::Matrix<double, 1, 1> R;
        R(0, 0) = 0.5 * 0.5;

        // Control matrix
        Eigen::Matrix<double, 2, 1> B;
        B[0] = 0.5 * 1.0 * 1.0;
        B[1] = 1.0;

        kf_.SetStateEstimate(x, P);
        kf_.SetTransitionMatrix(F);
        kf_.SetTransitionNoise(Q);
        kf_.SetObservationMatrix(H);
        kf_.SetObservationNoise(R);
        kf_.SetControlMatrix(B);
    }

protected:
    KalmanFilter<double, 2, 1, 1> kf_;
};

TEST_F(KalmanFilterTest, SyntheticTrackingTest)
{
    kf_.Predict();
    Eigen::Matrix<double, 2, 1> state = kf_.GetStateEstimate();
    Eigen::Matrix<double, 2, 2> state_cov = kf_.GetStateCovariance();
    EXPECT_DOUBLE_EQ(1.0, state(0, 0));
    EXPECT_DOUBLE_EQ(1.0, state(1, 0));
    EXPECT_NEAR(0.21, state_cov(0, 0), 0.001);
    EXPECT_NEAR(0.10, state_cov(0, 1), 0.001);
    EXPECT_NEAR(0.10, state_cov(1, 0), 0.001);
    EXPECT_NEAR(0.11, state_cov(1, 1), 0.001);

    Eigen::Matrix<double, 1, 1> z;
    z(0, 0) = 1.0;
    kf_.Correct(z);
    state = kf_.GetStateEstimate();
    state_cov = kf_.GetStateCovariance();

    EXPECT_DOUBLE_EQ(1.0, state(0, 0));
    EXPECT_DOUBLE_EQ(1.0, state(1, 0));
    EXPECT_NEAR(0.11413, state_cov(0, 0), 0.001);
    EXPECT_NEAR(0.05348, state_cov(0, 1), 0.001);
    EXPECT_NEAR(0.05348, state_cov(1, 0), 0.001);
    EXPECT_NEAR(0.08826, state_cov(1, 1), 0.001);
}
