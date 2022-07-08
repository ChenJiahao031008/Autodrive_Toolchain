#include <iostream>
#include <iomanip>

#include "common/logger.hpp"
#include "common/time_base.hpp"
#include "common/time_utils.hpp"
#include "interface/DataReader.h"
#include "interface/DataProcessor.h"
#include "interface/DataConverter.h"

int main(int argc, char** argv)
{
    common::Logger logger(argc, argv);

    // 测试文件读写接口使用 //
    std::ifstream pose_stream("../config_data/lidar_pose.txt", std::ios::in);
    if (!pose_stream.is_open())
    {
        AWARN << "[ERROR] Open File Failed.";
        return -1;
    }

    AINFO << "[INFO] Open File Success";
    auto vPoses = interface::GetTUMPoseFromFile(pose_stream);
    for (auto &pose: vPoses)
        AINFO << std::setprecision(15) << pose.timestamp;

    auto filenames = interface::ReadFolder("../config_data");
    for (auto &f : filenames)
        AINFO << f;

    if (interface::CopyDirectory("../config_data", "../resluts"))
    {
        if (interface::CopyFiles("../resluts/lidar_pose.txt", "../resluts/lidar_pose2.txt"))
        {
            auto filenames_1 = interface::ReadFolder("../resluts");
            for (auto &f : filenames_1)
                AINFO << f;
        }
    }
    std::string res_str;
    interface::GetFileNameInPath("../resluts/lidar_pose.txt", res_str);
    AINFO << "res_str: " << res_str;

    // 测试循环队列使用 //
    sensorData::CircularQueue<int> queue(10);
    for (int i = 0; i < 20; i++)
        queue.Enque(i);
    AINFO << queue;
    for (int i = 0; i < 10; i++)
        AINFO << queue.Deque() << " ";
    for (int i = 0; i < 15; i++)
        queue.Enque(i);
    AINFO << queue.front() << std::endl;
    AINFO << queue.rear() << std::endl;

    AINFO << "Copy Constructor Test." << std::endl;
    sensorData::CircularQueue<int> new_queue(queue);
    AINFO << new_queue;

    AINFO << "Move Constructor Test." << std::endl;
    sensorData::CircularQueue<int> new2_queue(std::move(queue));
    AINFO << new2_queue;

    // 测试旋转位姿插值使用 //
    auto pose_example_1 = vPoses[0].GetRotation();
    auto pose_example_2 = vPoses[30].GetRotation();
    auto pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8);
    AINFO << "pose1 : " << pose_example_1.coeffs();
    AINFO << "pose2 : " << pose_example_2.coeffs();
    AINFO << "Case 1 -------------------------------------";
    AINFO << "pose res : " << pose_result.coeffs();
    pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8, interface::interpolation_options::eigen_slerp);
    AINFO << "Case 2 -------------------------------------";
    AINFO << "pose res : " << pose_result.coeffs();
    pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8, interface::interpolation_options::slerp);
    AINFO << "Case 3 -------------------------------------";
    AINFO << "pose res : " << pose_result.coeffs();
    pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8, interface::interpolation_options::lie_group_interpolation);
    AINFO << "Case 4 -------------------------------------";
    AINFO << "pose res : " << pose_result.coeffs();

    // * 测试时间
    AINFO << "Time Now: " << common::Time::Now().ToString();
    auto t1 = common::Time::Now();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto t2 = common::Time::Now();
    AINFO << "During Time: " << (t2 - t1).ToMilliosecond();
    AINFO << "Clock Now: " << common::Clock::Now().ToString();
    common::Clock::SetNow(common::Time(1000));
    // todo: Clock理论上支持仿真时钟，但是代码并没有实现
    AINFO << "Change Now: " << common::Clock::Now().ToString();

    // 测试定时器模式 //
    int count = 0;
    common::Timer timer(
        100, [&count] { count = 100; }, true);
    timer.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    AINFO << "count start : " << count;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    timer.Stop();
    AINFO << "count end 2 : " << count;

    count = 0;
    common::Timer timer2( 2, [&count]{ AINFO << count++; }, false);
    timer2.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    AINFO << "count 1: " << count;
    timer2.Stop();
    AINFO << "count finshed: " << count;

    return 0;
}
