#pragma once

#include "rgbd_message_flow.h"

class RGBDIMessageFlow : public MessageFlow
{
private:
    bool initIMUFlag;
    bool delayed;
    int gravity_aixs;
    // 与图像进行同步的IMU数据缓存队列
    std::deque<sensor_msgs::Imu> synced_imu_data_buff_;
    // 未与图像进行同步的原始的IMU数据缓存队列
    std::deque<sensor_msgs::Imu> unsynced_imu_data_buff_;
    // 彩色图像队列
    std::deque<sensor_msgs::Image> image_color_data_buff_;
    // 深度图像队列
    std::deque<sensor_msgs::Image> image_depth_data_buff_;

    std::deque<sensor_msgs::Imu> unsynced_imu_data_;

    sensor_msgs::Imu synced_imu_data_;
    sensor_msgs::Image current_image_color_data_;
    sensor_msgs::Image current_image_depth_data_;
    cv::Mat cvColorImgMat, cvDepthMat;

public:
    RGBDIMessageFlow(ros::NodeHandle &nh, ROS_Interface_Config &config);

    ~RGBDIMessageFlow() override;

    void Run() override;

    bool ReadData() override;

    bool HasData() override;

    bool ValidData() override;

    bool InitIMU();

    bool IMUSyncData(
        std::deque<sensor_msgs::Imu> &UnsyncedDataBuff,
        std::deque<sensor_msgs::Imu> &UnsyncedData,
        std::deque<sensor_msgs::Imu> &SyncedData,
        ros::Time sync_time);
};
