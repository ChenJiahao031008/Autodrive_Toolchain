#pragma once

#include <fstream>
#include "../sensor_data/pose_data.hpp"
#include "../common/logger.hpp"

namespace interface
{

enum class save_options : unsigned char{
    TUM = 0, TUM_INVERSE = 1, KITTI = 2, KITTI_INVERSE = 3
};

class DataWriter
{
public:
    static void SaveTrajectory(const std::string &filename, const std::vector<sensorData::PoseData> &trajectory, save_options mode = save_options::TUM);
};

}
