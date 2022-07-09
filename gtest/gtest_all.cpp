#include <iostream>
#include <iomanip>
#include <gtest/gtest.h>

#include "../common/logger.hpp"
#include "../common/time_base.hpp"
#include "../common/time_utils.hpp"
#include "../interface/DataReader.h"
#include "../interface/DataProcessor.h"
#include "../interface/DataConverter.h"

TEST(TestDataReader, TestGetPose)
{
    // 测试文件读写接口使用，读取lidar_pose.txt文件并获取位姿
    std::ifstream pose_stream("../gtest/config_data/lidar_pose.txt", std::ios::in);
    auto vPoses = interface::GetTUMPoseFromFile(pose_stream);
    EXPECT_EQ(vPoses.size(), 953);
    EXPECT_DOUBLE_EQ(vPoses[0].timestamp, 1648022167.1589715);
    EXPECT_DOUBLE_EQ(vPoses[vPoses.size() - 1].timestamp, 1648022365.8215065);
    vPoses.clear();
    std::ifstream pose_stream2("../gtest/config_data/lidar_pose.txt", std::ios::in);
    vPoses = interface::GetTUMPoseFromFile(pose_stream2, 3);
    EXPECT_EQ(vPoses.size(), 950);
}

TEST(TestDataReader, FolderOperation)
{
    std::vector<std::string> filenames;
    std::string path0 = "../gtest/config_data/lidar_pose.txt";
    std::string path1 = "../gtest/resluts/test";
    std::string path2 = "../gtest/resluts/test/test2";
    std::string path3 = "../gtest/resluts/test/test2/lidar_pose2.txt";
    std::string path4 = "../gtest/resluts/test3/";
    std::string path5 = "../gtest/resluts/test4/";

    // 执行文件拷贝操作，目标文件夹不存在时会自动创建(文件->文件)
    interface::CopyFiles(path0, path3);
    // 执行文件拷贝操作，目标文件夹不存在时会自动创建(文件->文件夹)
    interface::CopyFiles(path0, path5);

    // mode=0:不允许递归, 输出文件和文件夹;
    interface::ReadFolder(path5, filenames, 0);
    EXPECT_EQ(filenames[0], path5 + "lidar_pose.txt");
    filenames.clear();

    // mode=1:允许递归, 但是只输出文件名;
    interface::ReadFolder(path1, filenames, 1);
    EXPECT_EQ(filenames[0], path3);
    filenames.clear();

    // mode=2:允许递归, 但是只输出文件夹;
    interface::ReadFolder(path1, filenames, 2);
    EXPECT_EQ(filenames[0], path2);
    filenames.clear();

    // mode=3: 允许递归, 输出文件名和文件路径;
    interface::ReadFolder(path1, filenames, 3);
    sort(filenames.begin(), filenames.end());
    EXPECT_EQ(filenames[0], path2);
    EXPECT_EQ(filenames[1], path3);
    filenames.clear();

    // 执行文件夹拷贝操作，目标文件夹不存在时会自动创建(文件夹->文件夹)
    interface::CopyFiles("../gtest/config_data", path4);
    interface::ReadFolder(path4, filenames, 1);
    sort(filenames.begin(), filenames.end());
    EXPECT_EQ(filenames[0], "../gtest/resluts/test3/extrinsics.yaml");
    EXPECT_EQ(filenames[1], "../gtest/resluts/test3/lidar_pose.txt");
    EXPECT_EQ(filenames[2], "../gtest/resluts/test3/test/test233.txt");

    // 获取文件的名称，不包含路径和扩展名
    std::string res_str;
    interface::GetFileNameInPath(path3, res_str);
    EXPECT_EQ(res_str, "lidar_pose2");

}

TEST(TestSensorData, TestCircularQueue)
{
    // 测试循环队列的使用
    sensorData::CircularQueue<int> queue(10);
    // 检查队列是否为空
    EXPECT_EQ(queue.isEmpty(), true);
    // 入栈
    for (int i = 0; i < 20; i++)
        queue.Enque(i);
    // 检查首尾指针是否一致
    EXPECT_EQ(queue.front(),10);
    EXPECT_EQ(queue.rear(), 19);
    // 检查队列是否满了
    EXPECT_EQ(queue.isFull(), true);
    // 出栈并检查是否正确
    for (int i = 0; i < 10; i++){
        EXPECT_EQ(queue.Deque(), i+10);
    }
    for (int i = 0; i < 15; i++)
        queue.Enque(i);

    sensorData::CircularQueue<int> new_queue(queue);
    EXPECT_EQ(new_queue.front(), 5);
    EXPECT_EQ(new_queue.rear(), 14);

    sensorData::CircularQueue<int> new2_queue(std::move(queue));
    EXPECT_EQ(new2_queue.front(), 5);
    EXPECT_EQ(new2_queue.rear(), 14);
}

TEST(TestDataProcessor, TestPoseInterpolation)
{
    // 测试旋转位姿插值使用 //
    std::ifstream pose_stream("../gtest/config_data/lidar_pose.txt", std::ios::in);
    auto vPoses = interface::GetTUMPoseFromFile(pose_stream);
    auto pose_example_1 = vPoses[0].GetRotation();
    auto pose_example_2 = vPoses[30].GetRotation();
    auto pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8);
    EXPECT_NEAR(pose_result.x(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.y(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.z(), 0.0045181, 1e-6);
    EXPECT_NEAR(pose_result.w(), 0.999988, 1e-6);
    pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8, interface::interpolation_options::eigen_slerp);
    EXPECT_NEAR(pose_result.x(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.y(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.z(), 0.0045181, 1e-6);
    EXPECT_NEAR(pose_result.w(), 0.99999, 1e-6);
    pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8, interface::interpolation_options::slerp);
    EXPECT_NEAR(pose_result.x(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.y(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.z(), 0.00451811, 1e-6);
    EXPECT_NEAR(pose_result.w(), 0.99999, 1e-6);
    pose_result = interface::DataProcessor::RotationInterpolation(pose_example_1, pose_example_2, 0.8, interface::interpolation_options::lie_group_interpolation);
    EXPECT_NEAR(pose_result.x(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.y(), 0.0, 1e-6);
    EXPECT_NEAR(pose_result.z(), 0.00451811, 1e-6);
    EXPECT_NEAR(pose_result.w(), 0.99999, 1e-6);
}

TEST(TestCommon, TestTime)
{
    // Time Class 测试
    auto t1 = common::Time::Now();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto t2 = common::Time::Now();
    EXPECT_EQ((t2 - t1).ToMilliosecond(), 100);

    common::Time time(100UL);
    EXPECT_EQ(100UL, time.ToNanosecond());

    time = common::Time(1.1);
    EXPECT_EQ(1100000000UL, time.ToNanosecond());
    EXPECT_DOUBLE_EQ(1.1, time.ToSecond());

    time = common::Time(1, 1);
    EXPECT_EQ(1000000001UL, time.ToNanosecond());
    EXPECT_DOUBLE_EQ(1.000000001, time.ToSecond());

    common::Time time2(time);
    EXPECT_EQ(time, time2);

    common::Time t1_1(100);
    common::Duration d(200);
    common::Time t2_1(300);
    EXPECT_EQ(t1_1 + d, t2_1);
    EXPECT_EQ(t2_1 - d, t1_1);
    EXPECT_EQ(t1_1 += d, t2_1);

    // Duration Class 测试
    common::Duration duration = common::Duration(1.1);
    EXPECT_EQ(1100000000UL, duration.ToNanosecond());
    EXPECT_DOUBLE_EQ(1.1, duration.ToSecond());

    duration = common::Duration(1, 1);
    EXPECT_EQ(1000000001UL, duration.ToNanosecond());
    EXPECT_DOUBLE_EQ(1.000000001, duration.ToSecond());

    common::Duration duration_2(duration);
    EXPECT_EQ(duration, duration_2);

    common::Duration d1(100);
    common::Duration d2(200);
    common::Duration d3(300);
    EXPECT_EQ(d1 + d2, d3);
    EXPECT_EQ(d2, d1 * 2);
    EXPECT_EQ(d3 - d2, d1);
    EXPECT_EQ(d1 += d2, d3);
}

TEST(TestCommon, TestTimeR)
{
    int count = 0;
    common::Timer timer(100, [&count]{ count = 100; }, true);
    timer.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    EXPECT_EQ(count, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    timer.Stop();
    EXPECT_EQ(count, 100);

    count = 0;
    common::Timer timer2(2, [&count]{ count++;}, false);
    timer2.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    EXPECT_EQ(count, 45);
    timer2.Stop();
}

int main(int argc, char **argv)
{
    common::Logger logger(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
