#include <iostream>
#include <iomanip>
#include "modules/visualization/image_visualization.h"
#include "modules/image_processing/depth_filter/JBF.h"
#include "modules/image_processing/depth_filter/Kernel.h"
#include "modules/image_processing/image_processing.h"
#include "interface/DataReader.h"
#include "common/logger.hpp"

#ifndef LOG_CORE_DUMP_CAPTURE
#define BACKWARD_HAS_DW 1
#endif

using namespace std;
using namespace visualization;
using namespace image_processing;

int main(int argc, char **argv)
{
    // 获得深度图和彩色图
    std::string path = interface::DataReader::GetCurrentDir();
    AINFO << path;
    std::string depth_path = path + "/../examples/image_processing/testdata/depth.png";
    std::string color_path = path + "/../examples/image_processing/testdata/color.png";
    AINFO << depth_path;
    cv::Mat inDepth = cv::imread(depth_path, CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat inColor = cv::imread(color_path, CV_LOAD_IMAGE_UNCHANGED);

    IMGVisualization::DepthMapVisualization(inDepth, display_option::DEPTH_DETAIL, "depth_detail.png");

    // 深度图采用联合双边滤波
    image_processing::JBF jointBilateralFilter;
    cv::Mat output = jointBilateralFilter.Processor(inColor, inDepth);
    IMGVisualization::DepthMapVisualization(output, display_option::DLSPLAY_AND_WAIT, "after_filter.png");

    // 深度图采用快速深度补全算法
    Config conf;
    conf.app.maxDepth = 65535;
    conf.app.minDepth = 0;
    Kernel kernel(conf);
    cv::Mat result = kernel.FillInFast(output);
    IMGVisualization::DepthMapVisualization(result, display_option::DEPTH_DETAIL, "after_completion.png");

    // 采用基于深度图的显著性检测算法
    // cv::Mat depthROI = inDepth(cv::Rect2i(345,38,255,261));
    cv::Mat object = IMGProcessing::DepthSaliencyDetection(inDepth);
    IMGVisualization::DepthMapVisualization(object, display_option::DEPTH_DETAIL, "after_saliency_detection.png");
    return 0;
}
