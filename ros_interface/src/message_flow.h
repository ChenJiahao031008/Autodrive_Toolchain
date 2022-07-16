#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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
    Eigen::Matrix4d INIT_POSE = Eigen::Matrix4d::Identity();

public:
    MessageFlow(){};
    virtual ~MessageFlow(){};
    virtual void Run() = 0;
    virtual bool ReadData() = 0;
    virtual bool HasData() = 0;
    virtual bool ValidData() = 0;
    // void SaveTrajectory();
};

class SLAMExample
{
    SLAMExample(){};
    ~SLAMExample(){};
    void SystemRun(){};
};
