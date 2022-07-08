// 该文件的设计参考了 cyber 时间处理类
#pragma once

#include <chrono>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include "logger.hpp"
#include "macros.hpp"

namespace common {
/* 基础时钟类型, 记录时间间隔*/
class Duration
{
public:
    Duration() = default;
    explicit Duration(int64_t nanoseconds);
    explicit Duration(int nanoseconds);
    explicit Duration(double seconds);
    Duration(uint32_t seconds, uint32_t nanoseconds);
    Duration(const Duration &other);
    Duration &operator=(const Duration &other);
    ~Duration() = default;

    double ToSecond() const;
    int64_t ToMicrosecond() const;
    int64_t ToMilliosecond() const;
    int64_t ToNanosecond() const;

    bool IsZero() const;
    void Sleep() const;

    Duration operator+(const Duration &rhs) const;
    Duration operator-(const Duration &rhs) const;
    Duration operator-() const;
    Duration operator*(double scale) const;
    Duration &operator+=(const Duration &rhs);
    Duration &operator-=(const Duration &rhs);
    Duration &operator*=(double scale);
    bool operator==(const Duration &rhs) const;
    bool operator!=(const Duration &rhs) const;
    bool operator>(const Duration &rhs) const;
    bool operator<(const Duration &rhs) const;
    bool operator>=(const Duration &rhs) const;
    bool operator<=(const Duration &rhs) const;

private:
    int64_t nanoseconds_ = 0;
};

std::ostream &operator<<(std::ostream &os, const Duration &rhs);

/* 基础时钟类型,时间不可变. 对chrono库中的TimePoint进行了封装和重载 */
class Time{
public:
    static const Time MAX;
    static const Time MIN;
    Time() = default;
    explicit Time(uint64_t nanoseconds);
    explicit Time(int nanoseconds);
    explicit Time(double seconds);
    Time(uint32_t seconds, uint32_t nanoseconds);
    Time(const Time &other);
    Time &operator=(const Time &other);

    static Time Now();
    static Time MonoTime();

    static void SleepUntil(const Time &time);

    static void Sleep(const Time &time);

    double ToSecond() const;
    uint64_t ToMicrosecond() const;
    uint64_t ToMilliosecond() const;
    uint64_t ToNanosecond() const;


    std::string ToString() const;

    bool IsZero() const;

    Duration operator-(const Time &rhs) const;
    Time operator+(const Duration &rhs) const;
    Time operator-(const Duration &rhs) const;
    Time &operator+=(const Duration &rhs);
    Time &operator-=(const Duration &rhs);
    bool operator==(const Time &rhs) const;
    bool operator!=(const Time &rhs) const;
    bool operator>(const Time &rhs) const;
    bool operator<(const Time &rhs) const;
    bool operator>=(const Time &rhs) const;
    bool operator<=(const Time &rhs) const;

private:
    uint64_t nanoseconds_ = 0;
};

/* 进阶时钟类型, 时间可变, 对TIME进行了单例模式的封装和调用 */
class Clock
{
public:
    static constexpr int64_t PRECISION =
        std::chrono::system_clock::duration::period::den /
        std::chrono::system_clock::duration::period::num;

    /// PRECISION >= 1000000 means the precision is at least 1us.
    static_assert(PRECISION >= 1000000,
                  "The precision of the system clock should be at least 1 "
                  "microsecond.");

    static Time Now() { return Time::Now(); };
    static double NowInSeconds() { return Now().ToSecond(); };

    static void SetNow(const Time &now){
        auto clock = Instance();
        clock->mock_now_ = now;
    };

    static void SetNowInSeconds(const double seconds)
    {
        Clock::SetNow(Time(seconds));
    }

    DECLARE_SINGLETON(Clock)

private:
    Time mock_now_ = Time(0);

};

/* 进阶时钟类型, 频率调用, 常用于固定间隔触发任务 */
class Rate
{
public:
    explicit Rate(double frequency);
    explicit Rate(uint64_t nanoseconds);
    explicit Rate(const Duration &);
    void Sleep();
    void Reset();
    Duration CycleTime() const;
    Duration ExpectedCycleTime() const { return expected_cycle_time_; }

private:
    Time start_;
    Duration expected_cycle_time_;
    Duration actual_cycle_time_;
};

}  // namespace common
