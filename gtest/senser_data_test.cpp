#include <iomanip>
#include <gtest/gtest.h>

#include "../common/logger.hpp"
#include "../sensor_data/Infrastructure.hpp"
#include "../sensor_data/pose_data.hpp"

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
    EXPECT_EQ(queue.front(), 10);
    EXPECT_EQ(queue.rear(), 19);
    // 检查队列是否满了
    EXPECT_EQ(queue.isFull(), true);
    // 出栈并检查是否正确
    for (int i = 0; i < 10; i++)
    {
        EXPECT_EQ(queue.Deque(), i + 10);
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
