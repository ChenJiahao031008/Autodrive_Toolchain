#pragma once

#include "message_flow.h"

class RGBDMessageFlow : public MessageFlow
{
private:
    bool delayed;
    std::deque<sensor_msgs::Image> image_color_data_buff_;
    std::deque<sensor_msgs::Image> image_depth_data_buff_;
    sensor_msgs::Image current_image_color_data_;
    sensor_msgs::Image current_image_depth_data_;
    cv::Mat cvColorImgMat, cvDepthMat;

public:
    RGBDMessageFlow(ros::NodeHandle &nh, ROS_Interface_Config &config);

    ~RGBDMessageFlow() override;

    void Run() override;

    bool ReadData() override;

    bool HasData() override;

    bool ValidData() override;
};
