#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>
#include "Config.h"
#include "Kernel.h"

// #define DEBUG
#ifdef DEBUG
    #define CHECK_INFO(x) std::cout << "[DEBUG] " << x << std::endl;
    #define CHECK_INFO_2(x,y) std::cout << "[DEBUG] " << x << y << std::endl;
#else
    #define CHECK_INFO(x) //std::cout << x << std::endl;
    #define CHECK_INFO_2(x,y) //std::cout << "[DEBUG] " << x << y << std::endl;
#endif

namespace image_processing
{

Kernel::Kernel(Config &config): setting(config)
{
    // std::cout << "[INFO]\tmax depth: " << setting.app.maxDepth << ";\n\tfill type: " << setting.app.fillMode
    // << ";\n\textrapolate: " << setting.app.extrapolate << ";\n\tblur type: " << setting.app.blurType << std::endl;
}


cv::Mat Kernel::FillInFast(cv::Mat &depthMap){
    cv::Mat depth32F;
    if (depthMap.type() == CV_32FC1){
        depth32F = depthMap;
    }else{
        depthMap.convertTo(depth32F, CV_32FC1);
    }

    #ifdef DEBUG
        cv::FileStorage fswrite("../result/test.xml", cv::FileStorage::WRITE);// 新建文件，覆盖掉已有文件
        fswrite << "depth32F" << depth32F;
        fswrite.release();
    #endif

    {
        size_t nr = depth32F.rows;
        size_t nc = depth32F.cols;
        if(depth32F.isContinuous())
        {
            nr = 1;
            nc = nc * depth32F.rows * depth32F.channels();
        }
        for(size_t i=0; i<nr; i++)
        {
            float* inData = depth32F.ptr<float>(i);
            for(size_t j=0; j< nc ; j++)
            {
                if (*inData > setting.app.minDepth)
                    *inData = setting.app.maxDepth - *inData;
                inData++;
            }
        }
    }


    morphologyEx(depth32F, depth32F, cv::MORPH_DILATE, DIAMOND_KERNEL_5);
    morphologyEx(depth32F, depth32F, cv::MORPH_CLOSE, FULL_KERNEL_5);

    cv::Mat dilated32F;
    morphologyEx(depth32F, dilated32F, cv::MORPH_DILATE, FULL_KERNEL_7);

    {
        size_t nr = depth32F.rows;
        size_t nc = depth32F.cols;
        if(depth32F.isContinuous() && dilated32F.isContinuous())
        {
            nr = 1;
            nc= nc*depth32F.rows * depth32F.channels();
        }
        for(size_t i=0; i<nr; i++)
        {
            float* outData = depth32F.ptr<float>(i);
            const float* inData = dilated32F.ptr<float>(i);
            for(size_t j=0; j< nc ; j++)
            {
                if (*outData < setting.app.minDepth){
                    *outData = *inData;
                }
                outData++;
                inData++;
            }
        }
    }


    if (setting.app.extrapolate){
        cv::Mat maskDepth = cv::Mat::zeros(depth32F.size(), CV_8UC1);
        cv::threshold(depth32F, maskDepth, setting.app.minDepth, setting.app.maxDepth, cv::THRESH_BINARY);

        #ifdef DEBUG
            cv::Mat maskDepthShow;
            maskDepth.convertTo(maskDepthShow, CV_8UC1);
            cv::imshow("maskDepthShow",maskDepthShow);
            cv::waitKey(0);
            cv::FileStorage fswrite("../result/maskDepth.xml", cv::FileStorage::WRITE);// 新建文件，覆盖掉已有文件
            fswrite << "maskDepth" << maskDepth;
            fswrite.release();
        #endif


        double min=0, max=0;
        cv::Point minLoc, maxLoc;
        std::vector<int> maxPosList;
        std::vector<float> maxDepthList;

        for (size_t x=0; x< maskDepth.cols; ++x){
            cv::Mat pixelCol = maskDepth.col(x);
            cv::minMaxLoc(pixelCol, &min, &max, &minLoc, &maxLoc);
            float tmpDepth = depth32F.at<float>(cv::Point(x,maxLoc.y));
            maxPosList.emplace_back(maxLoc.y);
            maxDepthList.emplace_back(tmpDepth);
            #ifdef DEBUG
                // std::cout << maxLoc.y << "\t";
                // std::cout << tmpDepth << "\t";
            #endif
        }

        for (size_t x=0; x< maskDepth.cols; ++x){
            for (size_t k=0; k< maxPosList[x]; ++k){
                depth32F.at<float>(cv::Point(x,k)) = maxDepthList[x];
            }
        }

        cv::Mat dilated;
        morphologyEx(depth32F, dilated, cv::MORPH_DILATE, FULL_KERNEL_31);

        size_t nr = depth32F.rows;
        size_t nc = depth32F.cols;
        if(depth32F.isContinuous() && dilated.isContinuous())
        {
            nr = 1;
            nc= nc*depth32F.rows * depth32F.channels();
        }
        for(size_t i=0; i<nr; i++)
        {
            float* outData = depth32F.ptr<float>(i);
            const float* inData = dilated.ptr<float>(i);
            for(size_t j=0; j< nc ; j++)
            {
                if (*outData < setting.app.minDepth){
                    *outData = *inData;
                }
                outData++;
                inData++;
            }
        }
    }


    cv::medianBlur(depth32F, depth32F, 5);

    cv::Mat blurred;
    if (setting.app.blurType == "bilateral"){
        bilateralFilter(depth32F, blurred, 5, 1.5, 2.0);
        depth32F = blurred.clone();
    }else if (setting.app.blurType == "gaussian"){
        cv::GaussianBlur(depth32F, blurred, cv::Size(5, 5), 0);
        {
            size_t nr = depth32F.rows;
            size_t nc = depth32F.cols;
            if(depth32F.isContinuous() && blurred.isContinuous())
            {
                nr = 1;
                nc= nc*depth32F.rows * depth32F.channels();
            }
            for(size_t i=0; i<nr; i++)
            {
                float* outData = depth32F.ptr<float>(i);
                const float* inData = blurred.ptr<float>(i);
                for(size_t j=0; j< nc ; j++)
                {
                    if (*outData > setting.app.minDepth){
                        *outData = *inData;
                    }
                    outData++;
                    inData++;
                }
            }
        }

    }

    cv::Mat result16U = cv::Mat::zeros(depth32F.size(), CV_16UC1);
    {
        size_t nr = depth32F.rows;
        size_t nc = depth32F.cols;
        if(depth32F.isContinuous() && result16U.isContinuous())
        {
            nr = 1;
            nc = nc*depth32F.rows * depth32F.channels();
        }
        for(size_t i=0; i<nr; i++)
        {
            float* inData   = depth32F.ptr<float>(i);
            ushort* outData = result16U.ptr<ushort>(i);
            for(size_t j=0; j< nc ; j++)
            {
                if (*inData > setting.app.minDepth){
                    *inData = setting.app.maxDepth - *inData;
                }

                *outData = static_cast<ushort>((*inData));
                inData++;
                outData++;
            }
        }
    }

    return result16U;
}

}
