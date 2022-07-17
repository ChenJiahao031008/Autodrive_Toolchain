#include "rgbd_message_flow.h"

RGBDMessageFlow::RGBDMessageFlow(ros::NodeHandle &nh, ROS_Interface_Config &config)
    : delayed(config.delayed)
{
    // 初始化图像
    image_sub_ptr_ = std::make_shared<IMGSubscriber>(nh, config.camera_topic, config.depth_topic, 1000);

    // 读取参数文件
    const std::string current_dir = ros::package::getPath("ros_interface");
    const std::string param_file = current_dir + "/" + config.yaml_file;

    /* TODO: SLAM系统在此初始化 */
    slam_ptr_ = std::make_shared<SLAMExample>();

    AINFO << "SUCCESS TO READ PARAM!";
}

RGBDMessageFlow::~RGBDMessageFlow()
{
    image_sub_ptr_.reset();
}

void RGBDMessageFlow::Run()
{
    if (!ReadData())
        return;

    while (HasData())
    {
        if (!ValidData())
            continue;
        slam_ptr_->Process(cvColorImgMat, cvDepthMat, image_time);
    }
}

bool RGBDMessageFlow::ReadData()
{
    image_sub_ptr_->ParseData(image_color_data_buff_, image_depth_data_buff_);
    if (image_color_data_buff_.size() == 0 || image_depth_data_buff_.size() == 0)
        return false;
    return true;
}

bool RGBDMessageFlow::HasData()
{
    if (image_color_data_buff_.size() == 0)
        return false;
    if (image_depth_data_buff_.size() == 0)
        return false;
    return true;
}

bool RGBDMessageFlow::ValidData()
{

    if (delayed)
    {
        current_image_color_data_ = image_color_data_buff_.front();
        current_image_depth_data_ = image_depth_data_buff_.front();
        image_color_data_buff_.pop_front();
        image_depth_data_buff_.pop_front();
    }
    else
    {
        current_image_color_data_ = image_color_data_buff_.back();
        current_image_depth_data_ = image_depth_data_buff_.back();
        image_color_data_buff_.clear();
        image_depth_data_buff_.clear();
    }

    image_time = current_image_color_data_.timestamp;
    cvColorImgMat = current_image_color_data_.image;
    cvDepthMat = current_image_depth_data_.image;

    return true;
}
