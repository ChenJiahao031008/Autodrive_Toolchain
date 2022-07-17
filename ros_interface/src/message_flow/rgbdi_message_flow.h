#pragma once

#include "message_flow.h"
#include "../subscriber/image_subscriber.h"
#include "../subscriber/imu_subscriber.h"

class RGBDIMessageFlow : public MessageFlow
{
private:
    bool delayed;
    bool init_imu_flag;

    int gravity_aixs;
    double image_time;

    std::shared_ptr<IMGSubscriber> image_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;

    std::deque<sensorData::IMGData> image_color_data_buff_;
    std::deque<sensorData::IMGData> image_depth_data_buff_;
    sensorData::IMGData current_image_color_data_;
    sensorData::IMGData current_image_depth_data_;

    sensorData::CircularQueue<sensorData::IMUData> synced_imu_data_buff_;
    sensorData::CircularQueue<sensorData::IMUData> unsynced_imu_data_buff_;
    sensorData::CircularQueue<sensorData::IMUData> unsynced_imu_data_;
    sensorData::IMUData synced_imu_data_;

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
        sensorData::CircularQueue<sensorData::IMUData> &UnsyncedDataBuff,
        sensorData::CircularQueue<sensorData::IMUData> &SyncedData,
        double sync_time, double max_interval = 0.05);
};
