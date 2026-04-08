#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <thread>

class RealtimeTimer {
public:
    enum class Mode {
        OneShot,
        Periodic,
    };

    using Callback = std::function<void()>;

    RealtimeTimer() noexcept;
    ~RealtimeTimer() noexcept;

    RealtimeTimer(const RealtimeTimer&) = delete;
    RealtimeTimer& operator=(const RealtimeTimer&) = delete;
    RealtimeTimer(RealtimeTimer&&) = delete;
    RealtimeTimer& operator=(RealtimeTimer&&) = delete;

    bool start(std::chrono::nanoseconds interval,
               Mode mode,
               Callback callback) noexcept;
    bool startOneShot(std::chrono::nanoseconds delay,
                      Callback callback) noexcept;
    bool startPeriodic(std::chrono::nanoseconds period,
                       Callback callback) noexcept;

    void stop() noexcept;

    bool active() const noexcept;
    const char* lastErrorString() const noexcept;

private:
    bool arm(std::chrono::nanoseconds interval, Mode mode) noexcept;
    void workerLoop() noexcept;
    void setLastError(const char* error) noexcept;

    int timer_fd_;
    int wake_fd_;
    std::thread worker_;
    Callback callback_;
    std::atomic<bool> active_;
    std::atomic<bool> stop_requested_;
    Mode mode_;
    mutable std::mutex mutex_;
    const char* last_error_;
};
