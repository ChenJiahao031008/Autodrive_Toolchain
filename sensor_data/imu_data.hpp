#pragma once

#include <deque>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace sensorData
{

class IMUData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp = 0.0;

    Eigen::Quaternionf orientation = Eigen::Quaternionf(Eigen::Matrix3f::Identity());
    Eigen::Vector3f linear_acceleration = Eigen::Vector3f::Zero();
    Eigen::Vector3f angular_velocity = Eigen::Vector3f::Zero();

    Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();
    Eigen::Vector3f accel_bias = Eigen::Vector3f::Zero();

public:

    inline Eigen::Matrix3f GetOrientationMatrix() const
    {
        return orientation.toRotationMatrix();
    };

};

}
