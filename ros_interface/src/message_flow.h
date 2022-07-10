#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../../common/logger.hpp"
#include "../../common/time_base.hpp"
#include "../../common/time_utils.hpp"
#include "../../config/config.h"
#include "../../sensor_data/pose_data.hpp"
#include "../../sensor_data/Infrastructure.hpp"
#include "../../interface/DataReader.h"
#include "../../interface/DataProcessor.h"
#include "../../interface/DataConverter.h"

#include <interpreter.hpp>

#include "imu_subscriber.h"
#include "image_subscriber.h"



class MessageFlow
{
protected:
    // 图像数据指针
    std::shared_ptr<IMGSubscriber> image_sub_ptr_;
    // IMU数据指针
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    // SLAM系统指针
    // std::shared_ptr<ORB_SLAM2::System> slam_ptr_;
public:
    MessageFlow(){};
    ~MessageFlow(){};
    virtual void Run() = 0;
    virtual bool ReadData() = 0;
    virtual bool HasData() = 0;
    virtual bool ValidData() = 0;
    // void SaveTrajectory();
};

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
    RGBDIMessageFlow(ros::NodeHandle &nh, ::ROS_Interface_Config &config);

    ~RGBDIMessageFlow();

    virtual void Run() override;

    virtual bool ReadData() override;

    virtual bool HasData() override;

    virtual bool ValidData() override;

    bool InitIMU();

    bool IMUSyncData(
        std::deque<sensor_msgs::Imu> &UnsyncedDataBuff,
        std::deque<sensor_msgs::Imu> &UnsyncedData,
        std::deque<sensor_msgs::Imu> &SyncedData,
        ros::Time sync_time);
};

class RGBDMessageFlow : public MessageFlow
{
private:
    bool initIMUFlag;
    bool delayed;
    int gravity_aixs;
    int count = 0;
    std::deque<sensor_msgs::Image> image_color_data_buff_;
    std::deque<sensor_msgs::Image> image_depth_data_buff_;
    sensor_msgs::Image current_image_color_data_;
    sensor_msgs::Image current_image_depth_data_;
    cv::Mat cvColorImgMat, cvDepthMat;

public:
    RGBDMessageFlow(ros::NodeHandle &nh, ROS_Interface_Config &config);

    ~RGBDMessageFlow();

    virtual void Run() override;

    virtual bool ReadData() override;

    virtual bool HasData() override;

    virtual bool ValidData() override;
};
