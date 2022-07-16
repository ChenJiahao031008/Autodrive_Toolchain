#pragma once
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "../../sensor_data/imu_data.hpp"
#include "../../sensor_data/Infrastructure.hpp"

class IMUSubscriber
{
public:
    IMUSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(sensorData::CircularQueue<sensorData::IMUData> &deque_imu_data);
    void ParseData(std::deque<sensor_msgs::Imu> &deque_imu_data);

private:
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<sensor_msgs::Imu> new_imu_data_;

    std::mutex buff_mutex_;
};
