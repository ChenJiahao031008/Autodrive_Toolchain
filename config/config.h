#pragma once
#include <string>
#include <vector>

struct Config
{
    std::string example_name;
};

struct Config_Test
{
    std::string example_name;
    double example_number;
    std::vector<int> example_vector;
};

struct ROS_Interface_Config
{
    int equipped_sensor;
    // rostopic channel:
    std::string camera_topic;
    std::string depth_topic;
    std::string imu_topic;
    std::string lidar_topic;
    std::string odom_topic;
    std::string gps_topic;
    // sensor param channel:
    std::string yaml_file;
    int gravity_aixs;
    bool delayed;
};
