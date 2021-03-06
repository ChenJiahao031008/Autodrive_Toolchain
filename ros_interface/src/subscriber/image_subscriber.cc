#include "image_subscriber.h"


IMGSubscriber::IMGSubscriber(ros::NodeHandle &nh, std::string color_topic_name, std::string depth_topic_name, size_t buff_size)
    : it_(nh)
{
    image_sub_color_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image> >(nh, color_topic_name, buff_size);

    image_sub_depth_ptr = std::make_shared<message_filters::Subscriber<sensor_msgs::Image> >(nh, depth_topic_name, buff_size);

    sync_ptr = std::make_shared<message_filters::Synchronizer<SyncPolicy> >(SyncPolicy(2 * buff_size), *image_sub_color_ptr, *image_sub_depth_ptr);

    sync_ptr->registerCallback(boost::bind(&IMGSubscriber::image_callback, this, _1, _2));
}

// 回调函数
// 输入：RBG彩色图像和深度图像
void IMGSubscriber::image_callback(const sensor_msgs::ImageConstPtr &msgImg, const sensor_msgs::ImageConstPtr &msgDepth)
{
    new_color_data_.push_back(*msgImg);
    new_depth_data_.push_back(*msgDepth);

    buff_mutex_.unlock();
}

void IMGSubscriber::ParseData(std::deque<sensor_msgs::Image> &image_color_data_buff, std::deque<sensor_msgs::Image> &image_depth_data_buff)
{
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_color_data_.size() > 0)
    {
        image_color_data_buff.insert(image_color_data_buff.end(), new_color_data_.begin(), new_color_data_.end());
        new_color_data_.clear();

        image_depth_data_buff.insert(image_depth_data_buff.end(), new_depth_data_.begin(), new_depth_data_.end());
        new_depth_data_.clear();
    }

    buff_mutex_.unlock();
}

void IMGSubscriber::ParseData(std::deque<sensorData::IMGData> &image_color_data_buff, std::deque<sensorData::IMGData> &image_depth_data_buff)
{
    buff_mutex_.lock();

    if (new_color_data_.size() > 0)
    {
        cv_bridge::CvImagePtr cv_ptr;
        for (auto &img : new_color_data_)
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
            cv::Mat img_mat = cv_ptr->image;
            double timestamp = img.header.stamp.toSec();
            image_color_data_buff.emplace_back(timestamp, img_mat);
        }
        new_color_data_.clear();


        for (auto &img : new_depth_data_)
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat img_mat = cv_ptr->image;
            double timestamp = img.header.stamp.toSec();
            image_depth_data_buff.emplace_back(timestamp, img_mat);
        }
        new_depth_data_.clear();
    }

    buff_mutex_.unlock();
}
