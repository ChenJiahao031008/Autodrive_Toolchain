#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "message_flow.h"
#include "../subscriber/image_subscriber.h"

class RGBDMessageFlow : public MessageFlow
{
private:
    bool delayed;
    double image_time;
    // 图像数据指针
    std::shared_ptr<IMGSubscriber> image_sub_ptr_;
    std::deque<sensorData::IMGData> image_color_data_buff_;
    std::deque<sensorData::IMGData> image_depth_data_buff_;
    sensorData::IMGData current_image_color_data_;
    sensorData::IMGData current_image_depth_data_;
    cv::Mat cvColorImgMat, cvDepthMat;

public:
    RGBDMessageFlow(ros::NodeHandle &nh, ROS_Interface_Config &config);

    ~RGBDMessageFlow() override;

    void Run() override;

    bool ReadData() override;

    bool HasData() override;

    bool ValidData() override;
};
