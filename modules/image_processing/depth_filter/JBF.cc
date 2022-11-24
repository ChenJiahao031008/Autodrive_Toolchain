#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>
#include <chrono>
#include "JBF.h"

using namespace cv;
using namespace std;

namespace image_processing{
JBF::JBF()
{}


cv::Mat
JBF::PaddingColor(cv::Mat img, int kernel_size)
{
    cv::Mat img_smoothed = cv::Mat::zeros(img.rows+kernel_size-1, img.cols+kernel_size-1, CV_8UC3);
    for(int y=0;y<img.rows;y++){
        for(int x=0;x<img.cols;x++){
            img_smoothed.at<cv::Vec3b>(y+kernel_size/2, x+kernel_size/2) = img.at<cv::Vec3b>(y, x);
        }
    }
    return img_smoothed;
}

cv::Mat
JBF::PaddingDepth(cv::Mat img, int kernel_size)
{
    cv::Mat img_smoothed = cv::Mat::zeros(img.rows+kernel_size-1, img.cols+kernel_size-1, CV_16UC1);
    for(int y=0;y<img.rows;y++){
        for(int x=0;x<img.cols;x++){
            img_smoothed.at<unsigned short>(y+kernel_size/2, x+kernel_size/2) = img.at<unsigned short>(y, x);
        }
    }
    return img_smoothed;
}

unsigned short
JBF::_JointBilateralFilterInpainting(cv::Mat colorImage_padded, cv::Mat depthImage_padded, int x, int y, int kernel_size, double sigma_pos, double sigma_col, double sigma_depth)
{
    double _kernel_var, kernel_var, W;
    _kernel_var=0;
    W=0;

    for(int k_y=-kernel_size/2; k_y<=kernel_size/2; k_y++){
        for(int k_x=-kernel_size/2; k_x<=kernel_size/2; k_x++){
            if(depthImage_padded.at<unsigned short>(y+k_y,x+k_x)!=0){
                cv::Vec3b centor_col = colorImage_padded.at<cv::Vec3b>(y,x);
                cv::Vec3b perf_col = colorImage_padded.at<cv::Vec3b>(y+k_y,x+k_x);
                unsigned short centor_depth = depthImage_padded.at<unsigned short>(y,x);
                unsigned short perf_depth = depthImage_padded.at<unsigned short>(y+k_y,x+k_x);

                double diff_pos = std::sqrt(double(k_x)*double(k_x) + double(k_y)*double(k_y));
                double diff_col_r = double(centor_col[0]) - double(perf_col[0]);
                double diff_col_g = double(centor_col[1]) - double(perf_col[1]);
                double diff_col_b = double(centor_col[2]) - double(perf_col[2]);
                double diff_col = std::sqrt(diff_col_r*diff_col_r + diff_col_g*diff_col_g + diff_col_b*diff_col_b);
                double _diff_depth = double(centor_depth) - double(perf_depth);
                double diff_depth = std::sqrt(_diff_depth*_diff_depth);
                double kernel_pos = std::exp(-diff_pos*diff_pos/(2*sigma_pos*sigma_pos));
                double kernel_col = std::exp(-diff_col*diff_col/(2*sigma_col*sigma_col));
                double kernel_depth = std::exp(-diff_depth*diff_depth/(2*sigma_depth*sigma_depth));
                _kernel_var += kernel_pos * kernel_col * kernel_depth * double(perf_depth);
                W += kernel_pos * kernel_col*kernel_depth;
            }
        }
    }

    if(W >0){
        kernel_var = static_cast<unsigned short>(_kernel_var/(W));
    }else{
        kernel_var = 0;
    }

    return kernel_var;
}

cv::Mat
JBF::JointBilateralFilterInpaintingOMP(cv::Mat colorImage, cv::Mat depthImage, vector<vector<int>> &inpainting_index, int kernel_size, double sigma_pos, double sigma_col, double sigma_depth)
{
    cv::Mat depthImage_smoothed = cv::Mat::zeros(depthImage.rows, depthImage.cols, CV_16U);
    cv::Mat depthImage_padded = PaddingDepth(depthImage, kernel_size);
    cv::Mat colorImage_padded = PaddingColor(colorImage, kernel_size);
    cv::Mat depthImage_mask = cv::Mat::zeros(depthImage.rows, depthImage.cols, CV_8UC1);

    cv::parallel_for_(cv::Range(0, depthImage.rows*depthImage.cols), [&](const cv::Range& range){
        for (int r = range.start; r < range.end; r++)
        {
            int y = r / depthImage.cols;
            int x = r % depthImage.cols;
            if(depthImage.at<unsigned short>(y, x) == 0){
                unsigned short kernel_var = _JointBilateralFilterInpainting(colorImage_padded, depthImage_padded, x+kernel_size/2, y+kernel_size/2, kernel_size, sigma_pos, sigma_col, sigma_depth);
                depthImage_smoothed.at<unsigned short>(y, x) = kernel_var;
                if(kernel_var>0){
                    depthImage_mask.at<unsigned char>(y, x) = 255;
                }
            }else{
                depthImage_smoothed.at<unsigned short>(y, x) = depthImage.at<unsigned short>(y, x);
            }
        }
    });

    for(int y=0;y<depthImage.rows;y++){
        for(int x=0;x<depthImage.cols;x++){
            if(depthImage_mask.at<unsigned char>(y, x)>0){
                vector<int> _inpainting_index;
                _inpainting_index.push_back(x);
                _inpainting_index.push_back(y);
                inpainting_index.push_back(_inpainting_index);
            }
        }
    }

    return depthImage_smoothed;
}

cv::Mat
JBF::RefineInpaintingArea(cv::Mat color, cv::Mat depthImage, vector<vector<int>> inpainting_index)
{
    cv::Mat _depth = depthImage.clone();

    cv::ximgproc::amFilter(color, depthImage, _depth, 5, 0.01, true);
    for(size_t i=0;i<inpainting_index.size();i++){
        vector<int> _inpainting_index = inpainting_index[i];
        int x = _inpainting_index[0];
        int y = _inpainting_index[1];
        depthImage.at<unsigned short>(_inpainting_index[1],_inpainting_index[0]) = _depth.at<unsigned short>(_inpainting_index[1],_inpainting_index[0]);
    }
    return depthImage;
}



cv::Mat JBF::Processor(cv::Mat &colorImage, cv::Mat &depthImage)
{
    std::chrono::system_clock::time_point  start, end;
    start = std::chrono::system_clock::now();

    vector<vector<int>> inpainting_index;
    Mat depthImage_smoothed = JointBilateralFilterInpaintingOMP(colorImage, depthImage, inpainting_index, 3, 2.0, 3.0, 100.0);
    for(int i=0;i<2;i++){
        depthImage_smoothed = JointBilateralFilterInpaintingOMP(colorImage, depthImage_smoothed, inpainting_index, 3, 2.0, 3.0, 100.0);
    }
    depthImage_smoothed = RefineInpaintingArea(colorImage, depthImage_smoothed, inpainting_index);
    end = std::chrono::system_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    // cout << elapsed << endl;

    return depthImage_smoothed;
}

void JBF::GetDepthColor(cv::Mat &depthImage)
{
    depthImage_color = cv::Mat::zeros( depthImage.rows, depthImage.cols, CV_8UC3);
    depthImage.convertTo( depthImage8, CV_8U, 255.0 / 4096 );
    cv::applyColorMap(depthImage8, depthImage_color, COLORMAP_JET);

    cv::imshow("depthImage", depthImage_color);
    cv::waitKey(0);
    cv::imwrite("../result/depthImage.png", depthImage_color);
}


void JBF::GetDepthSmoothedColor(cv::Mat &depthImage_smoothed)
{
    depthImage_color_smoothed = cv::Mat::zeros( depthImage_smoothed.rows, depthImage_smoothed.cols, CV_8UC3);
    depthImage_smoothed.convertTo( depthImage8, CV_8U, 255.0 / 4096 );
    cv::applyColorMap(depthImage8, depthImage_color_smoothed, COLORMAP_JET);

    cv::imshow("depthImageSmoothed", depthImage_color_smoothed);
    cv::waitKey(0);
    cv::imwrite("../result/depthImageSmoothed.png", depthImage_color_smoothed);
}

}
