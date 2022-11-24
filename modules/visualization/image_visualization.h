#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>

namespace visualization
{
    enum class display_option : unsigned char
    {
        DISPLAY = 0,
        DLSPLAY_AND_WAIT = 1,
        DLSPLAY_AND_SAVE = 2,
        DISPLAY_WAIT_AND_SAVE = 3,
        ONLY_SAVE = 4,
        DEPTH_DETAIL = 5,
    };

    class IMGVisualization
    {
    public:
        static void DepthMapVisualization(const cv::Mat &depth, display_option option = display_option::ONLY_SAVE, const std::string name = "");
    };
}
