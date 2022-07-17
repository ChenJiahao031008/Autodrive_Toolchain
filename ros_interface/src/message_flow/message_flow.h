#pragma once

#include <ros/ros.h>


#include "../../../common/logger.hpp"
#include "../../../common/time_base.hpp"
#include "../../../common/time_utils.hpp"
#include "../../../config/config.h"
#include "../../../sensor_data/image_data.hpp"
#include "../../../sensor_data/imu_data.hpp"
#include "../../../sensor_data/pose_data.hpp"
#include "../../../sensor_data/Infrastructure.hpp"
#include "../../../interface/DataReader.h"
#include "../../../interface/DataProcessor.h"
#include "../../../interface/DataConverter.h"
#include "../../../interface/DataWriter.h"

#include <interpreter.hpp>


class SLAMExample
{
private:
    std::vector<sensorData::PoseData> trajectory;

public:
    SLAMExample(){};

    ~SLAMExample() = default;

    void Process(cv::Mat &color, cv::Mat &depth, double timestamp){
        AINFO << "Processing data: " <<  std::setprecision(16) << timestamp; };

    void Process(cv::Mat &color, cv::Mat &depth, double timestamp, sensorData::IMUData &imu)
    {
        AINFO << "Processing data: " << std::setprecision(16) << imu.timestamp;
    };

    void SaveTrajectory(const std::string &filename)
    {
        AINFO << "Saving trajectory";
        interface::DataWriter::SaveTrajectory(filename, trajectory, interface::save_options::TUM);
    };
};

class MessageFlow
{
protected:
    // SLAM系统指针
    std::shared_ptr<SLAMExample> slam_ptr_;

    Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();

public:
    MessageFlow(){};

    virtual ~MessageFlow(){ slam_ptr_.reset(); };

    inline void SaveTrajectory(const std::string &filename){
        slam_ptr_->SaveTrajectory(filename);
    };

    virtual void Run() = 0;

    virtual bool ReadData() = 0;

    virtual bool HasData() = 0;

    virtual bool ValidData() = 0;

};


