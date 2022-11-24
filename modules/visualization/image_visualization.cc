#include "image_visualization.h"

namespace visualization
{
    void Mouse_Callback(int event, int x, int y, int flags, void *param)
    {
        cv::Mat *image_ptr = static_cast<cv::Mat *>(param);
        if (event == CV_EVENT_LBUTTONDOWN)
        {
            std::cout << "[INFO] Coord: "
                      << "x = " << x << "; y = " << y;
            cv::Point2f Points_Show;
            Points_Show.x = x;
            Points_Show.y = y;
            float inData = image_ptr->ptr<ushort>((int)Points_Show.y)[(int)Points_Show.x];
            std::cout << "; Depth: " << inData << std::endl;
        }
    }

    void IMGVisualization::DepthMapVisualization(const cv::Mat &depth, display_option option, const std::string name)
    {
        cv::Mat depthShow, depth8U;
        double min, max;
        cv::minMaxIdx(depth, &min, &max);
        convertScaleAbs(depth, depth8U, 255.0 / max);
        applyColorMap(depth8U, depthShow, cv::COLORMAP_JET);
        if (option == display_option::ONLY_SAVE || option == display_option::DLSPLAY_AND_SAVE)
        {
            cv::imwrite(name, depthShow);
        }

        if (option == display_option::DISPLAY || option == display_option::DLSPLAY_AND_SAVE)
        {
            cv::imshow(name, depthShow);
            cv::waitKey(1);
        }else if (option == display_option::DLSPLAY_AND_WAIT || option == display_option::DISPLAY_WAIT_AND_SAVE)
        {
            cv::imshow(name, depthShow);
            cv::waitKey(0);
        }else if (option == display_option::DEPTH_DETAIL)
        {
            cvNamedWindow(name.c_str());
            cvSetMouseCallback(name.c_str(), Mouse_Callback, (void *)&depth);
            cv::imshow(name, depthShow);
            cv::waitKey(0);
        }

    }

}
