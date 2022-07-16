#pragma once

#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <proj_api.h>

#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H

namespace sensorData
{
class GNSSData
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double timestamp = 0.0;
    // longitude, latitude, altitude
    Eigen::Vector3f LLT = Eigen::Vector3f::Zero();
    // north, east, up
    Eigen::Vector3f ENU = Eigen::Vector3f::Zero();
    // zone id
    int zone_id = 50;
    // gps_quality
    int status = 0;

public:
    // TODO: LLT到ENU的转换

};

}
