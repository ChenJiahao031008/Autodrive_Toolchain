#pragma once

#include "../sensor_data/pose_data.hpp"
#include "../common/logger.hpp"

namespace interface
{

// 旋转插值算法所用的方法
enum class interpolation_options : unsigned char
{
    nlerp = 0,
    slerp = 1,
    eigen_slerp = 2,
    lie_group_interpolation = 3
};

class DataProcessor
{
public:
// 函数功能：旋转插值算法，支持nlerp,slerp,eigen_slerp,lie_group_interpolation
// !注意：这个比例系数t对应q2的时间戳，而不是q1的时间戳
static Eigen::Quaternionf RotationInterpolation(
    Eigen::Quaternionf &q1,
    Eigen::Quaternionf &q2,
    float t,
    interpolation_options option = interpolation_options::nlerp);

};

// 函数功能：对两个时间戳之间的姿态进行插值，返回插值后的姿态
template <class T>
bool PoseSyncData(sensorData::CircularQueue<T> &unsynced_data, sensorData::CircularQueue<T> &synced_data, double sync_time, double max_interval = 0.2)
{
    if (unsynced_data.size() == 0)
        return false;

    auto index = unsynced_data.d_rear;
    while (index != unsynced_data.d_front)
    {
        if (unsynced_data.d_arr[index].GetTimestamp() < sync_time)
            break;

        if (index == 0)
            index = unsynced_data.d_maxsize - 1;
        else
            index--;
    }
    auto front_data = unsynced_data.d_arr[index];
    auto back_data = unsynced_data.DataPrev(index);

    double front_time = front_data.GetTimestamp();
    Eigen::Vector3f front_posi = front_data.GetPosition();
    Eigen::Quaternionf front_rot = front_data.GetRotation();

    double back_time = back_data.GetTimestamp();
    Eigen::Vector3f back_posi = back_data.GetPosition();
    Eigen::Quaternionf back_rot = back_data.GetRotation();

    if (back_time - sync_time > max_interval || sync_time - front_time > max_interval)
        return false;

    float front_scale = (back_time - sync_time) / (back_time - front_time);
    float back_scale = (sync_time - front_time) / (back_time - front_time);

    Eigen::Vector3f sync_posi = front_posi * front_scale + back_posi * back_scale;
    Eigen::Vector3f sync_lin_velo = front_data.GetVelocity() * front_scale + back_data.GetVelocity() * back_scale;
    Eigen::Vector3f sync_ang_velo = front_data.GetAngularVelocity() * front_scale + back_data.GetAngularVelocity() * back_scale;

    Eigen::Quaternionf sync_rot = DataProcessor::RotationInterpolation(front_rot, back_rot, back_scale);

    sensorData::PoseData sync_data(sync_time, sync_posi, sync_rot, sync_lin_velo, sync_ang_velo);
    synced_data.Enque(sync_data);

    return true;
}

}
