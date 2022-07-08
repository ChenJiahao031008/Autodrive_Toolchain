#include "DataProcessor.h"

namespace interface
{

Eigen::Quaternionf DataProcessor::RotationInterpolation(
    Eigen::Quaternionf &q1,
    Eigen::Quaternionf &q2,
    float t,
    interpolation_options option)
{
    Eigen::Quaternionf q_interpolated;
    if (option == interpolation_options::nlerp)
    {
        q_interpolated.x() = q1.x() * (1 - t) + q2.x() * t;
        q_interpolated.y() = q1.y() * (1 - t) + q2.y() * t;
        q_interpolated.z() = q1.z() * (1 - t) + q2.z() * t;
        q_interpolated.w() = q1.w() * (1 - t) + q2.w() * t;
        q_interpolated.normalized();
    }
    else if (option == interpolation_options::eigen_slerp)
    {
        q_interpolated = q1.slerp(t, q2);
    }
    else if (option == interpolation_options::slerp)
    {
        Eigen::Quaternionf q_diff = q2 * q1.inverse();
        float angle = 2 * acos(q_diff.w());
        Eigen::Vector3f axis = q_diff.vec() / sin(angle / 2);
        q_interpolated = Eigen::Quaternionf(Eigen::AngleAxisf(angle * t, axis)) * q1;
    }
    else if (option == interpolation_options::lie_group_interpolation){
        Eigen::Matrix3f matrix_diff = (q2 * q1.inverse()).matrix();
        sensorData::SO3<float> so3_diff(matrix_diff);
        decltype(auto) so3_interpolated = sensorData::SO3<float>::exp(t * so3_diff.log());
        q_interpolated = Eigen::Quaternionf(so3_interpolated.matrix()) * q1;
    }
    return q_interpolated;
}


}
