#include "RealtimeTimer.hpp"

#include <cerrno>
#include <cstdint>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr const char* kTimerNoError = "NONE";
constexpr const char* kTimerInvalidInterval = "invalid interval";
constexpr const char* kTimerUnavailable = "timer resources unavailable";
constexpr const char* kTimerSettimeFailed = "timerfd_settime failed";
constexpr const char* kTimerThreadStartFailed = "failed to start timer thread";
constexpr const char* kTimerPollFailed = "timer poll failed";
constexpr const char* kTimerReadFailed = "timer read failed";

timespec toTimespec(std::chrono::nanoseconds duration) noexcept {
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    const auto nanoseconds = duration - seconds;

    timespec spec{};
    spec.tv_sec = static_cast<time_t>(seconds.count());
    spec.tv_nsec = static_cast<long>(nanoseconds.count());
    return spec;
}

void drainEventFd(int fd) noexcept {
    if (fd < 0) {
        return;
    }

    std::uint64_t value = 0;
    while (read(fd, &value, sizeof(value)) == sizeof(value)) {
    }
}

void disarmTimerFd(int timer_fd) noexcept {
    if (timer_fd < 0) {
        return;
    }

    const itimerspec disarmed{};
    (void)timerfd_settime(timer_fd, 0, &disarmed, nullptr);
}

}  // namespace

RealtimeTimer::RealtimeTimer() noexcept
    : timer_fd_(timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC)),
      wake_fd_(eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK)),
      active_(false),
      stop_requested_(false),
      mode_(Mode::OneShot),
      last_error_(kTimerNoError) {
    if (timer_fd_ < 0 || wake_fd_ < 0) {
        setLastError(kTimerUnavailable);
    }
}

RealtimeTimer::~RealtimeTimer() noexcept {
    stop();
    if (timer_fd_ >= 0) {
        close(timer_fd_);
    }
    if (wake_fd_ >= 0) {
        close(wake_fd_);
    }
}

bool RealtimeTimer::start(std::chrono::nanoseconds interval,
                          Mode mode,
                          Callback callback) noexcept {
    stop();

    if (interval.count() <= 0) {
        setLastError(kTimerInvalidInterval);
        return false;
    }
    if (timer_fd_ < 0 || wake_fd_ < 0) {
        setLastError(kTimerUnavailable);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback_ = std::move(callback);
        mode_ = mode;
    }

    stop_requested_.store(false, std::memory_order_relaxed);

    if (!arm(interval, mode)) {
        std::lock_guard<std::mutex> lock(mutex_);
        callback_ = Callback{};
        return false;
    }

    try {
        worker_ = std::thread(&RealtimeTimer::workerLoop, this);
    } catch (...) {
        disarmTimerFd(timer_fd_);
        std::lock_guard<std::mutex> lock(mutex_);
        callback_ = Callback{};
        setLastError(kTimerThreadStartFailed);
        return false;
    }

    active_.store(true, std::memory_order_relaxed);
    setLastError(kTimerNoError);
    return true;
}

bool RealtimeTimer::startOneShot(std::chrono::nanoseconds delay,
                                 Callback callback) noexcept {
    return start(delay, Mode::OneShot, std::move(callback));
}

bool RealtimeTimer::startPeriodic(std::chrono::nanoseconds period,
                                  Callback callback) noexcept {
    return start(period, Mode::Periodic, std::move(callback));
}

void RealtimeTimer::stop() noexcept {
    stop_requested_.store(true, std::memory_order_relaxed);
    disarmTimerFd(timer_fd_);

    if (wake_fd_ >= 0) {
        const std::uint64_t wake_signal = 1;
        (void)write(wake_fd_, &wake_signal, sizeof(wake_signal));
    }

    if (worker_.joinable()) {
        worker_.join();
    }

    drainEventFd(wake_fd_);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback_ = Callback{};
    }

    active_.store(false, std::memory_order_relaxed);
}

bool RealtimeTimer::active() const noexcept {
    return active_.load(std::memory_order_relaxed);
}

const char* RealtimeTimer::lastErrorString() const noexcept {
    return last_error_;
}

bool RealtimeTimer::arm(std::chrono::nanoseconds interval, Mode mode) noexcept {
    itimerspec spec{};
    spec.it_value = toTimespec(interval);
    if (mode == Mode::Periodic) {
        spec.it_interval = toTimespec(interval);
    }

    if (timerfd_settime(timer_fd_, 0, &spec, nullptr) != 0) {
        setLastError(kTimerSettimeFailed);
        return false;
    }

    return true;
}

void RealtimeTimer::workerLoop() noexcept {
    pollfd fds[2]{};
    fds[0].fd = timer_fd_;
    fds[0].events = POLLIN;
    fds[1].fd = wake_fd_;
    fds[1].events = POLLIN;

    while (!stop_requested_.load(std::memory_order_relaxed)) {
        const int poll_result = poll(fds, 2, -1);
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            setLastError(kTimerPollFailed);
            break;
        }

        if ((fds[1].revents & POLLIN) != 0) {
            drainEventFd(wake_fd_);
            break;
        }

        if ((fds[0].revents & POLLIN) == 0) {
            continue;
        }

        std::uint64_t expirations = 0;
        if (read(timer_fd_, &expirations, sizeof(expirations)) !=
            static_cast<ssize_t>(sizeof(expirations))) {
            if (!stop_requested_.load(std::memory_order_relaxed)) {
                setLastError(kTimerReadFailed);
            }
            break;
        }

        Callback callback_copy;
        Mode mode_copy = Mode::OneShot;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            callback_copy = callback_;
            mode_copy = mode_;
        }

        if (callback_copy) {
            callback_copy();
        }

        if (mode_copy == Mode::OneShot) {
            break;
        }
    }

    disarmTimerFd(timer_fd_);
    active_.store(false, std::memory_order_relaxed);
}

void RealtimeTimer::setLastError(const char* error) noexcept {
    last_error_ = error;
}
