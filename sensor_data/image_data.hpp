#pragma once

#include <deque>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>

namespace sensorData
{

class IMGData
{
public:
    double timestamp = 0.0;
    cv::Mat image;

public:
    IMGData(){};

    IMGData(double timestamp, cv::Mat &image)
    {
        this->timestamp = timestamp;
        this->image = std::move(image);
    };
};


}; // namespace sensorData
