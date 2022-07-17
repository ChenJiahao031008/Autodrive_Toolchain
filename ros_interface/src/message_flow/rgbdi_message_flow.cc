#include "rgbdi_message_flow.h"

RGBDIMessageFlow::RGBDIMessageFlow(ros::NodeHandle &nh, ROS_Interface_Config &config)
    : init_imu_flag(false), gravity_aixs(config.gravity_aixs), delayed(config.delayed)
{
    // 初始化RGBD
    image_sub_ptr_ = std::make_shared<IMGSubscriber>(nh, config.camera_topic, config.depth_topic, 1000);
    // 初始化IMU
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, config.imu_topic, 500000);

    // 读取参数文件
    const std::string current_dir = ros::package::getPath("ros_interface");
    const std::string param_file = current_dir + "/" + config.yaml_file;

    /* TODO: SLAM系统在此初始化 */
    slam_ptr_ = std::make_shared<SLAMExample>();

    AINFO << "SUCCESS TO READ PARAM!";
}

RGBDIMessageFlow::~RGBDIMessageFlow()
{
    image_sub_ptr_.reset();
    imu_sub_ptr_.reset();
}

void RGBDIMessageFlow::Run()
{
    if (!ReadData())
        return;

    while (HasData())
    {
        if (!ValidData())
            continue;
        if (!InitIMU())
            continue;

        /* TODO: 系统在此进行跟踪 */
        slam_ptr_->Process(cvColorImgMat, cvDepthMat, image_time, synced_imu_data_);
    }
}

bool RGBDIMessageFlow::ReadData()
{
    image_sub_ptr_->ParseData(image_color_data_buff_, image_depth_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_data_buff_);

    if (image_color_data_buff_.size() == 0 || image_depth_data_buff_.size() == 0)
        return false;

    double current_time;
    if (delayed)
        current_time = image_color_data_buff_.front().timestamp;
    else
        current_time = image_color_data_buff_.back().timestamp;

    bool valid_imu = IMUSyncData(
        unsynced_imu_data_buff_, synced_imu_data_buff_, current_time);

    // only mark lidar as 'inited' when all the three sensors are synced:
    static bool sensor_inited = false;
    if (!sensor_inited)
    {
        if (!valid_imu)
        {
            std::cerr << "[WARNNIGN] FAIL TO START." << std::endl;
            if (delayed)
            {
                image_color_data_buff_.pop_front();
                image_depth_data_buff_.pop_front();
            }
            else
            {
                image_color_data_buff_.pop_back();
                image_depth_data_buff_.pop_back();
            }
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

bool RGBDIMessageFlow::IMUSyncData(
    sensorData::CircularQueue<sensorData::IMUData> &UnsyncedDataBuff,
    sensorData::CircularQueue<sensorData::IMUData> &SyncedData,
    double sync_time, double max_interval)
{
    if (UnsyncedDataBuff.size() == 0)
        return false;

    auto index = UnsyncedDataBuff.d_rear;
    while (index != UnsyncedDataBuff.d_front)
    {
        if (UnsyncedDataBuff.d_arr[index].timestamp < sync_time)
            break;

        if (index == 0)
            index = UnsyncedDataBuff.d_maxsize - 1;
        else
            index--;
    }
    auto front_data = UnsyncedDataBuff.d_arr[index];
    auto back_data = UnsyncedDataBuff.DataPrev(index);

    double front_time = front_data.timestamp;
    Eigen::Quaternionf front_orientation = front_data.orientation;
    Eigen::Vector3f front_linear_acceleration = front_data.linear_acceleration;
    Eigen::Vector3f front_angular_velocity = front_data.angular_velocity;

    double back_time = back_data.timestamp;
    Eigen::Quaternionf back_orientation = back_data.orientation;
    Eigen::Vector3f back_linear_acceleration = back_data.linear_acceleration;
    Eigen::Vector3f back_angular_velocity = back_data.angular_velocity;

    if (back_time - sync_time > max_interval || sync_time - front_time > max_interval)
        return false;

    float front_scale = (back_time - sync_time) / (back_time - front_time);
    float back_scale = (sync_time - front_time) / (back_time - front_time);

    Eigen::Quaternionf sync_orientation = interface::DataProcessor::RotationInterpolation(
        front_orientation,
        back_orientation,
        back_scale,
        interface::interpolation_options::eigen_slerp);

    Eigen::Vector3f sync_linear_acceleration =
        front_linear_acceleration * front_scale + back_linear_acceleration * back_scale;

    Eigen::Vector3f sync_angular_velocity =
        front_angular_velocity * front_scale + back_angular_velocity * back_scale;

    sensorData::IMUData sync_data;
    sync_data.timestamp = sync_time;
    sync_data.orientation = sync_orientation;
    sync_data.linear_acceleration = sync_linear_acceleration;
    sync_data.angular_velocity = sync_angular_velocity;
    SyncedData.Enque(sync_data);
    return true;
}

bool RGBDIMessageFlow::HasData()
{
    if (image_color_data_buff_.size() == 0)
        return false;
    if (image_depth_data_buff_.size() == 0)
        return false;
    if (synced_imu_data_buff_.size() == 0)
        return false;
    return true;
}

bool RGBDIMessageFlow::ValidData()
{
    double current_time;
    if (delayed)
    {
        current_time = image_color_data_buff_.front().timestamp;
        current_image_color_data_ = image_color_data_buff_.front();
        current_image_depth_data_ = image_depth_data_buff_.front();
        image_color_data_buff_.pop_front();
        image_depth_data_buff_.pop_front();
    }
    else
    {
        current_time = image_color_data_buff_.back().timestamp;
        current_image_color_data_ = image_color_data_buff_.back();
        current_image_depth_data_ = image_depth_data_buff_.back();
        image_color_data_buff_.clear();
        image_depth_data_buff_.clear();
    }

    synced_imu_data_ = synced_imu_data_buff_.Deque();

    double diff_time = current_image_color_data_.timestamp - synced_imu_data_.timestamp;
    if (diff_time > 0.1)
        return false;

    image_time = current_image_color_data_.timestamp;
    cvColorImgMat = current_image_color_data_.image;
    cvDepthMat = current_image_depth_data_.image;

    return true;
}

bool RGBDIMessageFlow::InitIMU()
{
    if (init_imu_flag)
        return true;
    Eigen::Vector3f &initIMU = synced_imu_data_.linear_acceleration;

    Eigen::Vector3f alpha_1 = initIMU;
    Eigen::Vector3f alpha_2, alpha_3;
    if (gravity_aixs == 2)
    {
        alpha_2 = Eigen::Vector3f(1, 0, 0);
        alpha_3 = Eigen::Vector3f(0, 0, 1);
    }
    else if (gravity_aixs == 3)
    {
        alpha_2 = Eigen::Vector3f(1, 0, 0);
        alpha_3 = Eigen::Vector3f(0, 1, 0);
    }
    Eigen::Vector3f beta_1 = alpha_1.normalized();
    Eigen::Vector3f beta_2 = (alpha_2 - beta_1.dot(alpha_2) * 1.0 / beta_1.dot(beta_1) * beta_1).normalized();
    Eigen::Vector3f beta_3 = (alpha_3 - beta_1.dot(alpha_3) * 1.0 / beta_1.dot(beta_1) * beta_1 - beta_2.dot(alpha_3) * 1.0 / beta_2.dot(beta_2) * beta_2).normalized();
    Eigen::Matrix3f InitR = Eigen::Matrix3f::Identity();
    if (gravity_aixs == 2)
    {
        InitR.block<3, 1>(0, 0) = beta_2;
        InitR.block<3, 1>(0, 1) = beta_1;
        InitR.block<3, 1>(0, 2) = beta_3;
    }
    else if (gravity_aixs == 3)
    {
        InitR.block<3, 1>(0, 0) = beta_2;
        InitR.block<3, 1>(0, 1) = beta_3;
        InitR.block<3, 1>(0, 2) = beta_1;
    }
    assert(InitR.determinant() == 1 || InitR.determinant() == -1);
    if (InitR.determinant() == -1)
    {
        InitR = -1 * InitR;
        assert(InitR.determinant() == 1);
    }
    init_pose.block<3, 3>(0, 0) = InitR;
    init_imu_flag = true;
    return true;
}
