#include "time_utils.hpp"

namespace common
{
/* —————————————————————————— TimingWheel 类实现 —————————————————————————— */

void TimingWheel::Start() {
  std::lock_guard<std::mutex> lock(running_mutex_);
  if (!running_) {
    ADEBUG << "TimeWheel start ok";
    running_ = true;
    tick_thread_ = std::thread([this]() { this->TickFunc(); });
  }
}

void TimingWheel::Shutdown() {
  std::lock_guard<std::mutex> lock(running_mutex_);
  if (running_) {
    running_ = false;
    if (tick_thread_.joinable()) {
      tick_thread_.join();
    }
  }
}

void TimingWheel::Tick() {
  auto& bucket = work_wheel_[current_work_wheel_index_];
  {
    std::lock_guard<std::mutex> lock(bucket.mutex());
    auto ite = bucket.task_list().begin();
    while (ite != bucket.task_list().end()) {
      auto task = ite->lock();
      if (task) {
        ADEBUG << "index: " << current_work_wheel_index_
               << " timer id: " << task->timer_id_;
        auto* callback = reinterpret_cast<std::function<void()>*>(&(task->callback));
        if (running_)
        {
            (*callback)();
        }
      }
      ite = bucket.task_list().erase(ite);
    }
  }
}

void TimingWheel::AddTask(const std::shared_ptr<TimerTask>& task) {
  AddTask(task, current_work_wheel_index_);
}

void TimingWheel::AddTask(const std::shared_ptr<TimerTask>& task,
                          const uint64_t current_work_wheel_index) {
  if (!running_) {
    Start();
  }
  auto work_wheel_index = current_work_wheel_index +
                          static_cast<uint64_t>(std::ceil(
                              static_cast<double>(task->next_fire_duration_ms) /
                              TIMER_RESOLUTION_MS));
  if (work_wheel_index >= WORK_WHEEL_SIZE) {
    auto real_work_wheel_index = GetWorkWheelIndex(work_wheel_index);
    task->remainder_interval_ms = real_work_wheel_index;
    auto assistant_ticks = work_wheel_index / WORK_WHEEL_SIZE;
    if (assistant_ticks == 1 &&
        real_work_wheel_index < current_work_wheel_index_) {
      work_wheel_[real_work_wheel_index].AddTask(task);
      ADEBUG << "add task to work wheel. index :" << real_work_wheel_index;
    } else {
      auto assistant_wheel_index = 0;
      {
        std::lock_guard<std::mutex> lock(current_assistant_wheel_index_mutex_);
        assistant_wheel_index = GetAssistantWheelIndex(
            current_assistant_wheel_index_ + assistant_ticks);
        assistant_wheel_[assistant_wheel_index].AddTask(task);
      }
      ADEBUG << "add task to assistant wheel. index : "
             << assistant_wheel_index;
    }
  } else {
    work_wheel_[work_wheel_index].AddTask(task);
    ADEBUG << "add task [" << task->timer_id_
           << "] to work wheel. index :" << work_wheel_index;
  }
}

void TimingWheel::Cascade(const uint64_t assistant_wheel_index) {
  auto& bucket = assistant_wheel_[assistant_wheel_index];
  std::lock_guard<std::mutex> lock(bucket.mutex());
  auto ite = bucket.task_list().begin();
  while (ite != bucket.task_list().end()) {
    auto task = ite->lock();
    if (task) {
      work_wheel_[task->remainder_interval_ms].AddTask(task);
    }
    ite = bucket.task_list().erase(ite);
  }
}

void TimingWheel::TickFunc() {
  Rate rate(TIMER_RESOLUTION_MS * 1000000);  // ms to ns
  while (running_) {
    Tick();
    // AINFO_EVERY(1000) << "Tick " << TickCount();
    tick_count_++;
    rate.Sleep();
    {
      std::lock_guard<std::mutex> lock(current_work_wheel_index_mutex_);
      current_work_wheel_index_ =
          GetWorkWheelIndex(current_work_wheel_index_ + 1);
    }
    if (current_work_wheel_index_ == 0) {
      {
        std::lock_guard<std::mutex> lock(current_assistant_wheel_index_mutex_);
        current_assistant_wheel_index_ =
            GetAssistantWheelIndex(current_assistant_wheel_index_ + 1);
      }
      Cascade(current_assistant_wheel_index_);
    }
  }
}

TimingWheel::TimingWheel() {}

/* —————————————————————————— Time 类实现 —————————————————————————— */
namespace{
std::atomic<uint64_t> global_timer_id = {0};
uint64_t GenerateTimerId() { return global_timer_id.fetch_add(1); }
} // namespace

Timer::Timer()
{
    timing_wheel_ = TimingWheel::Instance();
    timer_id_ = GenerateTimerId();
}

Timer::Timer(TimerOption opt) : timer_opt_(opt)
{
    timing_wheel_ = TimingWheel::Instance();
    timer_id_ = GenerateTimerId();
}

Timer::Timer(uint32_t period, std::function<void()> callback, bool oneshot)
{
    timing_wheel_ = TimingWheel::Instance();
    timer_id_ = GenerateTimerId();
    timer_opt_.period = period;
    timer_opt_.callback = callback;
    timer_opt_.oneshot = oneshot;
}

void Timer::SetTimerOption(TimerOption opt) { timer_opt_ = opt; }

bool Timer::InitTimerTask()
{
    if (timer_opt_.period == 0)
    {
        AERROR << "Max interval must great than 0";
        return false;
    }

    if (timer_opt_.period >= TIMER_MAX_INTERVAL_MS)
    {
        AERROR << "Max interval must less than " << TIMER_MAX_INTERVAL_MS;
        return false;
    }

    task_.reset(new TimerTask(timer_id_));
    task_->interval_ms = timer_opt_.period;
    task_->next_fire_duration_ms = task_->interval_ms;
    if (timer_opt_.oneshot)
    {
        std::weak_ptr<TimerTask> task_weak_ptr = task_;
        task_->callback = [callback = this->timer_opt_.callback, task_weak_ptr]()
        {
            auto task = task_weak_ptr.lock();
            if (task)
            {
                std::lock_guard<std::mutex> lg(task->mutex);
                callback();
            }
        };
    }
    else
    {
        std::weak_ptr<TimerTask> task_weak_ptr = task_;
        task_->callback = [callback = this->timer_opt_.callback, task_weak_ptr]()
        {
            auto task = task_weak_ptr.lock();
            if (!task)
            {
                return;
            }
            std::lock_guard<std::mutex> lg(task->mutex);
            auto start = Time::MonoTime().ToNanosecond();
            callback();
            auto end = Time::MonoTime().ToNanosecond();
            uint64_t execute_time_ns = end - start;
            uint64_t execute_time_ms =
            std::llround(static_cast<double>(execute_time_ns) / 1e6);
            if (task->last_execute_time_ns == 0)
            {
                task->last_execute_time_ns = start;
            }
            else
            {
                task->accumulated_error_ns +=
                    start - task->last_execute_time_ns - task->interval_ms * 1000000;
            }
            ADEBUG << "start: " << start << "\t last: " << task->last_execute_time_ns
                   << "\t execut time:" << execute_time_ms
                   << "\t accumulated_error_ns: " << task->accumulated_error_ns;
            task->last_execute_time_ns = start;
            if (execute_time_ms >= task->interval_ms)
            {
                task->next_fire_duration_ms = TIMER_RESOLUTION_MS;
            }
            else
            {
                int64_t accumulated_error_ms = std::llround(
                    static_cast<double>(task->accumulated_error_ns) / 1e6);
                if (static_cast<int64_t>(task->interval_ms - execute_time_ms -
                                         TIMER_RESOLUTION_MS) >= accumulated_error_ms)
                {
                    task->next_fire_duration_ms =
                        task->interval_ms - execute_time_ms - accumulated_error_ms;
                }
                else
                {
                    task->next_fire_duration_ms = TIMER_RESOLUTION_MS;
                }
                ADEBUG << "error ms: " << accumulated_error_ms
                       << "  execute time: " << execute_time_ms
                       << " next fire: " << task->next_fire_duration_ms
                       << " error ns: " << task->accumulated_error_ns;
            }
            TimingWheel::Instance()->AddTask(task);
        };
    }
    return true;
}

void Timer::Start()
{

    if (!started_.exchange(true))
    {
        if (InitTimerTask())
        {
            timing_wheel_->AddTask(task_);
            AINFO << "start timer [" << task_->timer_id_ << "]";
        }
    }
}

void Timer::Stop()
{
    if (started_.exchange(false) && task_)
    {
        AINFO << "stop timer, the timer_id: " << timer_id_;
        // using a shared pointer to hold task_->mutex before task_ reset
        auto tmp_task = task_;
        {
            std::lock_guard<std::mutex> lg(tmp_task->mutex);
            task_.reset();
        }
    }
}

Timer::~Timer()
{
    if (task_)
    {
        Stop();
    }
}
}
