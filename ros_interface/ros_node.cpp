
#include "src/rgbd_message_flow.h"
#include "src/rgbdi_message_flow.h"

#ifndef LOG_CORE_DUMP_CAPTURE
#define BACKWARD_HAS_DW 1
#endif

enum class sensor_options : int
{
    CAMERA = 0,
    CAMERA_IMU = 2,
    CAMERA_DEPTH = 3,
    CAMERA_DEPTH_IMU = 6,
    LIDAR_IMU_GPS_FUSION = 10,
    LIDAR_IMU_GPS_ODOM_FUSION = 20
};

// bool save_odometry = false;
// bool SaveOdometryCb(saveOdometry::Request &request, saveOdometry::Response &response)
// {
//     save_odometry = true;
//     response.succeed = true;
//     return response.succeed;
// }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ros_interface");
    ros::NodeHandle nh;
    common::Logger logger(argc, argv);

    std::string current_ws = ros::package::getPath("ros_interface");
    AINFO << "Current Dir is: " << current_ws;
    ezcfg::Interpreter itp(current_ws + "/../config/ros_interface_config.txt", true);
    ROS_Interface_Config conf;
    itp.parse(conf);

    // ros::ServiceServer service = nh.advertiseService("save_odometry", SaveOdometryCb);
    std::shared_ptr<MessageFlow> message_flow_ptr;
    switch (static_cast<sensor_options>(conf.equipped_sensor))
    {
    case sensor_options::CAMERA_DEPTH:
        message_flow_ptr = std::make_shared<RGBDMessageFlow>(nh, conf);
        break;
    case sensor_options::CAMERA_DEPTH_IMU:
        message_flow_ptr = std::make_shared<RGBDIMessageFlow>(nh, conf);
        break;
    default:
        break;
    }

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        message_flow_ptr->Run();
        // if (save_odometry)
        // {
        //     save_odometry = false;
        //     // message_flow_ptr->SaveTrajectory();
        // }

        rate.sleep();
    }

    return 0;
}
