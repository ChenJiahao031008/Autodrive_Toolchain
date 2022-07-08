// 该文件参考cyber的时间处理类

#include "time_base.hpp"

namespace common {

using std::chrono::high_resolution_clock; // 获取高精度时间，不允许修改
using std::chrono::steady_clock;          // 秒表时钟，适合用于记录程序耗时
using std::chrono::system_clock;          // 获取系统时间，允许修改

const Time Time::MAX = Time(std::numeric_limits<uint64_t>::max());
const Time Time::MIN = Time(0);

Clock::Clock(){}
/* ———————————————————————— Time 类代码实现 ———————————————————————— */
Time::Time(uint64_t nanoseconds) { nanoseconds_ = nanoseconds; }

Time::Time(int nanoseconds)
{
    nanoseconds_ = static_cast<uint64_t>(nanoseconds);
}

Time::Time(double seconds)
{
    nanoseconds_ = static_cast<uint64_t>(seconds * 1000000000UL);
}

Time::Time(uint32_t seconds, uint32_t nanoseconds)
{
    nanoseconds_ = static_cast<uint64_t>(seconds) * 1000000000UL + nanoseconds;
}

Time::Time(const Time &other) { nanoseconds_ = other.nanoseconds_; }

Time &Time::operator=(const Time &other)
{
    this->nanoseconds_ = other.nanoseconds_;
    return *this;
}

Time Time::Now()
{
    // 获得time_point类型的时间
    auto now = high_resolution_clock::now();
    auto nano_time_point =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    // 将time_point转换为系统时间并输出
    auto epoch = nano_time_point.time_since_epoch();
    uint64_t now_nano =
        std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    return Time(now_nano);
}

Time Time::MonoTime()
{
    auto now = steady_clock::now();
    auto nano_time_point =
        std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = nano_time_point.time_since_epoch();
    uint64_t now_nano =
        std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();
    return Time(now_nano);
}

double Time::ToSecond() const
{
    return static_cast<double>(nanoseconds_) / 1000000000UL;
}

bool Time::IsZero() const { return nanoseconds_ == 0; }

uint64_t Time::ToNanosecond() const { return nanoseconds_; }

uint64_t Time::ToMicrosecond() const
{
    return static_cast<uint64_t>(nanoseconds_ / 1000.0);
}

uint64_t Time::ToMilliosecond() const
{
    return static_cast<uint64_t>(nanoseconds_ / 1000000.0);
}

std::string Time::ToString() const
{
    auto nano = std::chrono::nanoseconds(nanoseconds_);
    system_clock::time_point tp(nano);
    auto time = system_clock::to_time_t(tp);
    struct tm stm;
    auto ret = localtime_r(&time, &stm);
    if (ret == nullptr)
    {
        return std::to_string(static_cast<double>(nanoseconds_) / 1000000000.0);
    }

    std::stringstream ss;
#if __GNUC__ >= 5
    ss << std::put_time(ret, "%F %T");
    ss << "." << std::setw(9) << std::setfill('0') << nanoseconds_ % 1000000000UL;
#else
    char date_time[128];
    strftime(date_time, sizeof(date_time), "%F %T", ret);
    ss << std::string(date_time) << "." << std::setw(9) << std::setfill('0')
       << nanoseconds_ % 1000000000UL;
#endif
    return ss.str();
}

void Time::SleepUntil(const Time &time)
{
    auto nano = std::chrono::nanoseconds(time.ToNanosecond());
    system_clock::time_point tp(nano);
    std::this_thread::sleep_until(tp);
}

void Time::Sleep(const Time &time)
{
    auto sleep_time = std::chrono::nanoseconds(time.ToNanosecond());
    std::this_thread::sleep_for(sleep_time);
}

Duration Time::operator-(const Time &rhs) const
{
    return Duration(static_cast<int64_t>(nanoseconds_ - rhs.nanoseconds_));
}

Time Time::operator+(const Duration &rhs) const
{
    return Time(nanoseconds_ + rhs.ToNanosecond());
}

Time Time::operator-(const Duration &rhs) const
{
    return Time(nanoseconds_ - rhs.ToNanosecond());
}

Time &Time::operator+=(const Duration &rhs)
{
    *this = *this + rhs;
    return *this;
}

Time &Time::operator-=(const Duration &rhs)
{
    *this = *this - rhs;
    return *this;
}

bool Time::operator==(const Time &rhs) const
{
    return nanoseconds_ == rhs.nanoseconds_;
}

bool Time::operator!=(const Time &rhs) const
{
    return nanoseconds_ != rhs.nanoseconds_;
}

bool Time::operator>(const Time &rhs) const
{
    return nanoseconds_ > rhs.nanoseconds_;
}

bool Time::operator<(const Time &rhs) const
{
    return nanoseconds_ < rhs.nanoseconds_;
}

bool Time::operator>=(const Time &rhs) const
{
    return nanoseconds_ >= rhs.nanoseconds_;
}

bool Time::operator<=(const Time &rhs) const
{
    return nanoseconds_ <= rhs.nanoseconds_;
}

std::ostream &operator<<(std::ostream &os, const Time &rhs)
{
    os << rhs.ToString();
    return os;
}

/* ———————————————————————— Duration 类代码实现 ———————————————————————— */
Duration::Duration(int64_t nanoseconds) { nanoseconds_ = nanoseconds; }

Duration::Duration(int nanoseconds)
{
    nanoseconds_ = static_cast<int64_t>(nanoseconds);
}

Duration::Duration(double seconds)
{
    nanoseconds_ = static_cast<int64_t>(seconds * 1000000000UL);
}

Duration::Duration(uint32_t seconds, uint32_t nanoseconds)
{
    nanoseconds_ = static_cast<uint64_t>(seconds) * 1000000000UL + nanoseconds;
}

Duration::Duration(const Duration &other) { nanoseconds_ = other.nanoseconds_; }

Duration &Duration::operator=(const Duration &other)
{
    this->nanoseconds_ = other.nanoseconds_;
    return *this;
}

double Duration::ToSecond() const
{
    return static_cast<double>(nanoseconds_) / 1000000000UL;
}

int64_t Duration::ToNanosecond() const { return nanoseconds_; }

int64_t Duration::ToMicrosecond() const
{
    return static_cast<int64_t>(nanoseconds_ / 1000.0);
}

int64_t Duration::ToMilliosecond() const
{
    return static_cast<int64_t>(nanoseconds_ / 1000000.0);
}

bool Duration::IsZero() const { return nanoseconds_ == 0; }

void Duration::Sleep() const
{
    auto sleep_time = std::chrono::nanoseconds(nanoseconds_);
    std::this_thread::sleep_for(sleep_time);
}

Duration Duration::operator+(const Duration &rhs) const
{
    return Duration(nanoseconds_ + rhs.nanoseconds_);
}

Duration Duration::operator-(const Duration &rhs) const
{
    return Duration(nanoseconds_ - rhs.nanoseconds_);
}

Duration Duration::operator-() const { return Duration(-nanoseconds_); }

Duration Duration::operator*(double scale) const
{
    return Duration(int64_t(static_cast<double>(nanoseconds_) * scale));
}

Duration &Duration::operator+=(const Duration &rhs)
{
    *this = *this + rhs;
    return *this;
}

Duration &Duration::operator-=(const Duration &rhs)
{
    *this = *this - rhs;
    return *this;
}

Duration &Duration::operator*=(double scale)
{
    *this = Duration(int64_t(static_cast<double>(nanoseconds_) * scale));
    return *this;
}

bool Duration::operator==(const Duration &rhs) const
{
    return nanoseconds_ == rhs.nanoseconds_;
}

bool Duration::operator!=(const Duration &rhs) const
{
    return nanoseconds_ != rhs.nanoseconds_;
}

bool Duration::operator>(const Duration &rhs) const
{
    return nanoseconds_ > rhs.nanoseconds_;
}

bool Duration::operator<(const Duration &rhs) const
{
    return nanoseconds_ < rhs.nanoseconds_;
}

bool Duration::operator>=(const Duration &rhs) const
{
    return nanoseconds_ >= rhs.nanoseconds_;
}

bool Duration::operator<=(const Duration &rhs) const
{
    return nanoseconds_ <= rhs.nanoseconds_;
}

std::ostream &operator<<(std::ostream &os, const Duration &rhs)
{
    std::ios::fmtflags before(os.flags());
    os << std::fixed << std::setprecision(9) << rhs.ToSecond() << "s";
    os.flags(before);
    return os;
}

/* ———————————————————————— Rate 类代码实现 ———————————————————————— */

Rate::Rate(double frequency)
    : start_(Time::Now()),
      expected_cycle_time_(1.0 / frequency),
      actual_cycle_time_(0.0) {}

Rate::Rate(uint64_t nanoseconds)
    : start_(Time::Now()),
      expected_cycle_time_(static_cast<int64_t>(nanoseconds)),
      actual_cycle_time_(0.0) {}

Rate::Rate(const Duration &d)
    : start_(Time::Now()), expected_cycle_time_(d), actual_cycle_time_(0.0) {}

void Rate::Sleep()
{
    Time expected_end = start_ + expected_cycle_time_;

    Time actual_end = Time::Now();

    // detect backward jumps in time
    if (actual_end < start_)
    {
        AWARN << "Detect backward jumps in time";
        expected_end = actual_end + expected_cycle_time_;
    }

    // calculate the time we'll sleep for
    Duration sleep_time = expected_end - actual_end;

    // set the actual amount of time the loop took in case the user wants to kNow
    actual_cycle_time_ = actual_end - start_;

    // make sure to reset our start time
    start_ = expected_end;

    // if we've taken too much time we won't sleep
    if (sleep_time < Duration(0.0))
    {
        AWARN << "Detect forward jumps in time";
        // if we've jumped forward in time, or the loop has taken more than a full
        // extra cycle, reset our cycle
        if (actual_end > expected_end + expected_cycle_time_)
        {
            start_ = actual_end;
        }
        // return false to show that the desired rate was not met
        return;
    }

    Time::SleepUntil(expected_end);
}

void Rate::Reset() { start_ = Time::Now(); }

Duration Rate::CycleTime() const { return actual_cycle_time_; }

}  // namespace common
