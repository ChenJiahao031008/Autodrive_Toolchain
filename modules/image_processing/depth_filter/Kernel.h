#pragma once
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>
#include "Config.h"


#ifndef KERNEL_H
#define KERNEL_H
#endif

namespace image_processing
{
class Kernel
{
public:
    cv::Mat CROSS_KERNEL_3  = (cv::Mat_<uchar>(3,3) <<
        0, 1, 0,
        1, 1, 1,
        0, 1, 0
    );

    cv::Mat CROSS_KERNEL_5 = (cv::Mat_<uchar>(5,5) <<
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0,
        1, 1, 1, 1, 1,
        0, 0, 1, 0, 0,
        0, 0, 1, 0, 0
    );

    cv::Mat DIAMOND_KERNEL_5 = (cv::Mat_<uchar>(5,5) <<
        0, 0, 1, 0, 0,
        0, 1, 1, 1, 0,
        1, 1, 1, 1, 1,
        0, 1, 1, 1, 0,
        0, 0, 1, 0, 0
    );

    cv::Mat CROSS_KERNEL_7 = (cv::Mat_<uchar>(7,7) <<
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0,
        1, 1, 1, 1, 1, 1, 1,
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0
    );

    cv::Mat DIAMOND_KERNEL_7 = (cv::Mat_<uchar>(7,7) <<
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 1, 1, 1, 0, 0,
        0, 1, 1, 1, 1, 1, 0,
        1, 1, 1, 1, 1, 1, 1,
        0, 1, 1, 1, 1, 1, 0,
        0, 0, 1, 1, 1, 0, 0,
        0, 0, 0, 1, 0, 0, 0
    );

    cv::Mat FULL_KERNEL_3   = cv::Mat::ones(3, 3, CV_8UC1);
    cv::Mat FULL_KERNEL_5   = cv::Mat::ones(5, 5, CV_8UC1);
    cv::Mat FULL_KERNEL_7   = cv::Mat::ones(7, 7, CV_8UC1);
    cv::Mat FULL_KERNEL_9   = cv::Mat::ones(9, 9, CV_8UC1);
    cv::Mat FULL_KERNEL_31  = cv::Mat::ones(31, 31, CV_8UC1);

    Config setting;

public:

    Kernel(Config &config);

    cv::Mat FillInFast(cv::Mat &depthMap);

};


}
