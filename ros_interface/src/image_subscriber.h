#pragma once

#include <deque>
#include <mutex>
#include <thread>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/package.h>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include "image_subscriber.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// using namespace std;
// 近似同步器
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

class IMGSubscriber {
  public:
    IMGSubscriber(ros::NodeHandle &nh, std::string color_topic_name, std::string depth_topic_name, size_t buff_size);

    IMGSubscriber() = default;

    void ParseData(std::deque<sensor_msgs::Image> &image_color_data_buff, std::deque<sensor_msgs::Image> &image_depth_data_buff);

  private:
    void image_callback(const sensor_msgs::ImageConstPtr& msgImg,const sensor_msgs::ImageConstPtr& msgDepth);

  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_; // 图像转换

    std::deque<sensor_msgs::Image> new_color_data_;
    std::deque<sensor_msgs::Image> new_depth_data_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_color_ptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> image_sub_depth_ptr;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_ptr;

    std::mutex buff_mutex_;
};

