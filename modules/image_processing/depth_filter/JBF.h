#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>

namespace image_processing{
class JBF
{
public:
    JBF();

    ~JBF(){};

    cv::Mat PaddingColor(cv::Mat img, int kernel_size);

    cv::Mat PaddingDepth(cv::Mat img, int kernel_size);

    unsigned short _JointBilateralFilterInpainting(cv::Mat colorImage_padded, cv::Mat depthImage_padded,
        int x, int y, int kernel_size, double sigma_pos, double sigma_col, double sigma_depth);

    cv::Mat JointBilateralFilterInpaintingOMP(cv::Mat colorImage, cv::Mat depthImage,
        std::vector<std::vector<int>> &inpainting_index, int kernel_size, double sigma_pos, double sigma_col,
        double sigma_depth);

    cv::Mat RefineInpaintingArea(cv::Mat color, cv::Mat depthImage, std::vector<std::vector<int>> inpainting_index);

    cv::Mat Processor(cv::Mat &colorImage, cv::Mat &depthImage);

    void GetDepthColor(cv::Mat &depthImage);

    void GetDepthSmoothedColor(cv::Mat &depthImage_smoothed);

private:
    cv::Mat depthImage_color,depthImage_color_smoothed;

    cv::Mat depthImage8;

};

}
