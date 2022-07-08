#pragma once

#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <atomic>
#include <cmath>

#include "time_base.hpp"
#include "logger.hpp"

namespace common
{

struct TimerTask {
  explicit TimerTask(uint64_t timer_id) : timer_id_(timer_id) {}
  uint64_t timer_id_ = 0;
  std::function<void()> callback;
  uint64_t interval_ms = 0;
  uint64_t remainder_interval_ms = 0;
  uint64_t next_fire_duration_ms = 0;
  int64_t accumulated_error_ns = 0;
  uint64_t last_execute_time_ns = 0;
  std::mutex mutex;
};

class TimerBucket {
 public:
  void AddTask(const std::shared_ptr<TimerTask>& task) {
    std::lock_guard<std::mutex> lock(mutex_);
    task_list_.push_back(task);
  }

  std::mutex& mutex() { return mutex_; }
  std::list<std::weak_ptr<TimerTask>>& task_list() { return task_list_; }

 private:
  std::mutex mutex_;
  std::list<std::weak_ptr<TimerTask>> task_list_;
};

static const uint64_t WORK_WHEEL_SIZE = 512;
static const uint64_t ASSISTANT_WHEEL_SIZE = 64;
static const uint64_t TIMER_RESOLUTION_MS = 2;
static const uint64_t TIMER_MAX_INTERVAL_MS =
    WORK_WHEEL_SIZE * ASSISTANT_WHEEL_SIZE * TIMER_RESOLUTION_MS;

class TimingWheel
{
public:
    ~TimingWheel()
    {
        if (running_)
        {
            Shutdown();
        }
    }

    void Start();

    void Shutdown();

    void Tick();

    void AddTask(const std::shared_ptr<TimerTask> &task);

    void AddTask(const std::shared_ptr<TimerTask> &task,
                 const uint64_t current_work_wheel_index);

    void Cascade(const uint64_t assistant_wheel_index);

    void TickFunc();

    inline uint64_t TickCount() const { return tick_count_; }

private:
    inline uint64_t GetWorkWheelIndex(const uint64_t index)
    {
        return index & (WORK_WHEEL_SIZE - 1);
    }
    inline uint64_t GetAssistantWheelIndex(const uint64_t index)
    {
        return index & (ASSISTANT_WHEEL_SIZE - 1);
    }

    bool running_ = false;
    uint64_t tick_count_ = 0;
    std::mutex running_mutex_;
    TimerBucket work_wheel_[WORK_WHEEL_SIZE];
    TimerBucket assistant_wheel_[ASSISTANT_WHEEL_SIZE];
    uint64_t current_work_wheel_index_ = 0;
    std::mutex current_work_wheel_index_mutex_;
    uint64_t current_assistant_wheel_index_ = 0;
    std::mutex current_assistant_wheel_index_mutex_;
    std::thread tick_thread_;

    DECLARE_SINGLETON(TimingWheel)
};

struct TimerOption
{
    /**
     * @brief Construct a new Timer Option object
     *
     * @param period The period of the timer, unit is ms
     * @param callback The task that the timer needs to perform
     * @param oneshot Oneshot or period
     */
    TimerOption(uint32_t period, std::function<void()> callback, bool oneshot)
        : period(period), callback(callback), oneshot(oneshot) {}

    TimerOption() : period(), callback(), oneshot() {}

    // The period of the timer, unit is ms. max: 512 * 64, min: 1
    uint32_t period = 0;

    std::function<void()> callback;

    bool oneshot;
};


class Timer
{
public:
    Timer();

    explicit Timer(TimerOption opt);

    Timer(uint32_t period, std::function<void()> callback, bool oneshot);
    ~Timer();

    void SetTimerOption(TimerOption opt);

    void Start();

    void Stop();

private:
    bool InitTimerTask();
    uint64_t timer_id_;
    TimerOption timer_opt_;
    TimingWheel *timing_wheel_ = nullptr;
    std::shared_ptr<TimerTask> task_;
    std::atomic<bool> started_ = {false};
};


}
