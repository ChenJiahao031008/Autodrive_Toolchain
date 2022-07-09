#include <gtest/gtest.h>

#include "../common/logger.hpp"
#include "../common/time_base.hpp"
#include "../common/time_utils.hpp"

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

TEST(TestCommon, TestTimer)
{
    int count = 0;
    common::Timer timer(
        100, [&count]
        { count = 100; },
        true);
    timer.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    EXPECT_EQ(count, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    timer.Stop();
    EXPECT_EQ(count, 100);

    count = 0;
    common::Timer timer2(
        2, [&count]
        { count++; },
        false);
    timer2.Start();
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    EXPECT_EQ(count, 45);
    timer2.Stop();
}

TEST(TestCommon, TestUnix2Gps)
{
    double unix_time = 1476761767;
    double gps_time = common::TimeUtil::Unix2Gps(unix_time);
    EXPECT_NEAR(gps_time, 1160796984, 0.000001);

    double unix_time1 = 1483228799;
    double gps_time1 = common::TimeUtil::Unix2Gps(unix_time1);
    EXPECT_NEAR(gps_time1, 1167264017, 0.000001);
}

TEST(TestCommon, TestGps2Unix)
{
    double gps_time = 1160796984;
    double unix_time = common::TimeUtil::Gps2Unix(gps_time);
    EXPECT_NEAR(unix_time, 1476761767, 0.000001);
    double gps_time1 = 1260796984;
    double unix_time1 = common::TimeUtil::Gps2Unix(gps_time1);
    EXPECT_NEAR(unix_time1, 1576761766, 0.000001);
}
