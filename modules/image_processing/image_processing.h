#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ximgproc.hpp>

namespace image_processing{
    class IMGProcessing
    {
    public:
        static cv::Mat DepthSaliencyDetection(const cv::Mat &depth);
    };

}
