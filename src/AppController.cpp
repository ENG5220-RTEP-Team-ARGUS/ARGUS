#include "AppController.hpp"

#include "CameraCapture.hpp"
#include "CppTimerStdFuncCallback.h"
#include "GuardianStateMachine.hpp"
#include "PhysicalButtonModule.hpp"
#include "RobotInterlock.hpp"
#include "VisionProcessor.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr const char* kLiveCameraWindowName = "ARGUS Live Camera";
constexpr const char* kLiveStatusWindowName = "ARGUS Live Status";
constexpr const char* kLiveMetricsWindowName = "ARGUS Live Metrics";
constexpr const char* kDefaultI2cDevicePath = "/dev/i2c-1";
constexpr std::uint8_t kDefaultPca9685Address = 0x40;
constexpr float kDefaultPwmFrequencyHz = 50.0f;
constexpr std::uint8_t kBaseServoChannel = 0;
constexpr std::uint8_t kLowerServoChannel = 4;
constexpr std::uint8_t kUpperServoChannel = 8;
constexpr std::uint8_t kGripServoChannel = 12;
constexpr MotionChannelMap kMotionChannelMap{
    kBaseServoChannel,
    kLowerServoChannel,
    kUpperServoChannel,
    kGripServoChannel};
constexpr int kLiveFreezeBadFrameThreshold = 30;
constexpr int kLiveRecoverGoodFrameThreshold = 3;
constexpr std::chrono::milliseconds kSmokeStepDwell{3000};
constexpr int kSmokeBaseMinOffset = -90;
constexpr int kSmokeBaseMaxOffset = 90;
constexpr int kSmokeLowerMinOffset = -90;
constexpr int kSmokeLowerMaxOffset = 90;
constexpr int kSmokeUpperMinOffset = -90;
constexpr int kSmokeUpperMaxOffset = 90;
constexpr int kSmokeGripMinOffset = -90;
constexpr int kSmokeGripMaxOffset = 90;
constexpr int kSmokePositiveStep = 90;
constexpr int kSmokeNegativeStep = -90;
volatile std::sig_atomic_t g_interactive_servo_stop_requested = 0;
constexpr std::chrono::milliseconds kMotionHomeSettleDwell{2000};
constexpr std::chrono::milliseconds kDemoStepDwell{1000};
constexpr int kDemoBaseMinOffset = -45;
constexpr int kDemoBaseMaxOffset = 45;
constexpr int kDemoLowerMinOffset = -45;
constexpr int kDemoLowerMaxOffset = 45;
constexpr int kDemoUpperMinOffset = -45;
constexpr int kDemoUpperMaxOffset = 45;
constexpr int kDemoGripMinOffset = -90;
constexpr int kDemoGripMaxOffset = 90;
constexpr int kDemoBaseStep = 45;
constexpr int kDemoLowerStep = 45;
constexpr int kDemoUpperStep = 45;
constexpr int kDemoGripStep = 45;
constexpr std::chrono::milliseconds kCaptureRetryBackoff{50};
constexpr std::chrono::milliseconds kDemoSlewStepInterval{100};
constexpr int kDemoSlewStepDegrees = 1;
constexpr int kLiveManualNudgeDegrees = 5;
constexpr std::chrono::milliseconds kSurgeryRetractDwell{350};
constexpr int kSurgeryForwardOffset = 22;
constexpr int kSurgeryBackwardOffset = -22;
constexpr int kSurgeryPassOneDepthOffset = -12;
constexpr int kSurgeryPassTwoDepthOffset = -24;
constexpr int kSurgeryPassThreeDepthOffset = -36;
constexpr int kSurgeryGripHoldOffset = 90;

class MotionControllerHardwareAdapter final : public RobotHardware {
public:
    explicit MotionControllerHardwareAdapter(MotionController& motion_controller) noexcept
        : motion_controller_(motion_controller) {}

    bool freezeMotion() noexcept override {
        motion_controller_.freeze();
        if (motion_controller_.outputState() == MotionOutputState::FAULT) {
            std::cerr << "[MOTION] freezeMotion() failed: "
                      << motion_controller_.lastErrorString() << std::endl;
            return false;
        }
        return true;
    }

    bool enableMotion() noexcept override {
        if (!motion_controller_.enable()) {
            std::cerr << "[MOTION] enableMotion() failed: "
                      << motion_controller_.lastErrorString() << std::endl;
            return false;
        }
        return true;
    }

private:
    MotionController& motion_controller_;
};

void handleInteractiveServoSignal(int) {
    g_interactive_servo_stop_requested = 1;
}

bool waitForCppTimerDelay(std::chrono::nanoseconds delay,
                          std::string& error_message) {
    if (delay.count() <= 0) {
        error_message.clear();
        return true;
    }

    std::mutex mutex;
    std::condition_variable condition;
    bool fired = false;

    CppTimerCallback timer;
    timer.registerEventCallback([&]() {
        std::lock_guard<std::mutex> lock(mutex);
        fired = true;
        condition.notify_one();
    });

    try {
        timer.startns(static_cast<long>(delay.count()), ONESHOT);
    } catch (const char* exception) {
        error_message = exception;
        return false;
    } catch (...) {
        error_message = "CppTimer start failed";
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&]() { return fired; });
    timer.stop();
    error_message.clear();
    return true;
}

double computeFocusScore(const cv::Mat& image) {
    if (image.empty()) {
        return 0.0;
    }

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

const char* focusQualityLabel(double focus_score) {
    if (focus_score < 60.0) {
        return "BLURRY";
    }
    if (focus_score < 180.0) {
        return "SOFT";
    }
    return "SHARP";
}

std::string formatFocusScore(double focus_score) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << focus_score;
    return oss.str();
}

struct RuntimeLatencyMetrics {
    long long vision_us = 0;
    std::optional<long long> unsafe_detect_ms;
    std::optional<long long> freeze_pipeline_ms;
    std::optional<long long> freeze_cmd_ms;
    std::optional<long long> total_stop_ms;
    std::optional<long long> ack_to_resume_ms;
};

enum class ControllerEventKind {
    ButtonInput,
    DemoStepReady,
    LiveStepReady,
    FrameAvailable,
    FrameCaptureFailed,
};

enum class ControllerEventDisposition {
    Consumed,
    Deferred,
    Abort,
};

struct ControllerEvent {
    ControllerEventKind kind;
    std::optional<PhysicalButtonEvent> button_event;
    std::optional<FrameEvent> frame_event;
};

class ControllerEventQueue {
public:
    void pushButton(PhysicalButtonEvent button_event) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(ControllerEvent{
            ControllerEventKind::ButtonInput,
            button_event,
            std::nullopt,
        });
        condition_.notify_one();
    }

    void pushDemoStepReady() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (demo_step_pending_) {
            return;
        }
        demo_step_pending_ = true;
        queue_.push_back(ControllerEvent{
            ControllerEventKind::DemoStepReady,
            std::nullopt,
            std::nullopt,
        });
        condition_.notify_one();
    }

    void pushLiveStepReady() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (live_step_pending_) {
            return;
        }
        live_step_pending_ = true;
        queue_.push_back(ControllerEvent{
            ControllerEventKind::LiveStepReady,
            std::nullopt,
            std::nullopt,
        });
        condition_.notify_one();
    }

    void pushFrame(FrameEvent frame_event) {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(ControllerEvent{
            ControllerEventKind::FrameAvailable,
            std::nullopt,
            std::move(frame_event),
        });
        condition_.notify_one();
    }

    void pushFrameCaptureFailed() {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push_back(ControllerEvent{
            ControllerEventKind::FrameCaptureFailed,
            std::nullopt,
            std::nullopt,
        });
        condition_.notify_one();
    }

    bool waitForEvents(std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!queue_.empty()) {
            return true;
        }

        if (timeout.count() <= 0) {
            return false;
        }

        return condition_.wait_for(lock, timeout, [&]() { return !queue_.empty(); });
    }

    template <typename Handler>
    bool drain(Handler&& handler) {
        std::deque<ControllerEvent> drained;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            drained.swap(queue_);
            demo_step_pending_ = false;
            live_step_pending_ = false;
        }

        std::vector<ControllerEvent> deferred;
        deferred.reserve(drained.size());
        for (const ControllerEvent& event : drained) {
            const ControllerEventDisposition disposition = handler(event);
            if (disposition == ControllerEventDisposition::Abort) {
                return false;
            }
            if (disposition == ControllerEventDisposition::Deferred) {
                deferred.push_back(event);
            }
        }

        if (!deferred.empty()) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (const ControllerEvent& event : deferred) {
                switch (event.kind) {
                    case ControllerEventKind::ButtonInput:
                        queue_.push_back(event);
                        break;
                    case ControllerEventKind::DemoStepReady:
                        if (!demo_step_pending_) {
                            demo_step_pending_ = true;
                            queue_.push_back(event);
                        }
                        break;
                    case ControllerEventKind::LiveStepReady:
                        if (!live_step_pending_) {
                            live_step_pending_ = true;
                            queue_.push_back(event);
                        }
                        break;
                    case ControllerEventKind::FrameAvailable:
                    case ControllerEventKind::FrameCaptureFailed:
                        queue_.push_back(event);
                        break;
                }
            }
            condition_.notify_one();
        }

        return true;
    }

private:
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<ControllerEvent> queue_;
    bool demo_step_pending_ = false;
    bool live_step_pending_ = false;
};

long long elapsedMilliseconds(std::chrono::steady_clock::time_point from,
                              std::chrono::steady_clock::time_point to) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(to - from)
        .count();
}

std::string formatLatencyMilliseconds(const std::optional<long long>& value) {
    return value.has_value() ? std::to_string(*value) : "N/A";
}

void logLiveLatencySample(const char* event_label,
                          const RuntimeLatencyMetrics& metrics) {
    std::cout << "[LIVE_TEST] latency event=" << event_label
              << " vision_us=" << metrics.vision_us
              << " unsafe_detect_ms="
              << formatLatencyMilliseconds(metrics.unsafe_detect_ms)
              << " freeze_pipeline_ms="
              << formatLatencyMilliseconds(metrics.freeze_pipeline_ms)
              << " freeze_cmd_ms="
              << formatLatencyMilliseconds(metrics.freeze_cmd_ms)
              << " total_stop_ms="
              << formatLatencyMilliseconds(metrics.total_stop_ms)
              << " ack_to_resume_ms="
              << formatLatencyMilliseconds(metrics.ack_to_resume_ms)
              << std::endl;
}

int uiPlainFontFace() {
#ifdef CV_VERSION_MAJOR
    return cv::FONT_HERSHEY_PLAIN;
#else
    return cv::FONT_HERSHEY_SIMPLEX;
#endif
}

int frameWidth(const cv::Mat& frame) {
#ifdef CV_VERSION_MAJOR
    return frame.cols;
#else
    (void)frame;
    return 640;
#endif
}

int frameHeight(const cv::Mat& frame) {
#ifdef CV_VERSION_MAJOR
    return frame.rows;
#else
    (void)frame;
    return 480;
#endif
}

cv::Mat makePlaceholderFrame(int width, int height) {
#ifdef CV_VERSION_MAJOR
    return cv::Mat::zeros(height, width, CV_8UC3);
#else
    (void)width;
    (void)height;
    return cv::Mat();
#endif
}

void drawRectangle(cv::Mat& frame,
                   cv::Point top_left,
                   cv::Point bottom_right,
                   const cv::Scalar& color,
                   int thickness) {
#ifdef CV_VERSION_MAJOR
    cv::rectangle(frame, top_left, bottom_right, color, thickness);
#else
    (void)frame;
    (void)top_left;
    (void)bottom_right;
    (void)color;
    (void)thickness;
#endif
}

void drawLine(cv::Mat& frame,
              cv::Point from,
              cv::Point to,
              const cv::Scalar& color,
              int thickness) {
#ifdef CV_VERSION_MAJOR
    cv::line(frame, from, to, color, thickness);
#else
    (void)frame;
    (void)from;
    (void)to;
    (void)color;
    (void)thickness;
#endif
}

struct SupervisoryUiRow {
    std::string label;
    std::string value;
    cv::Scalar value_color;
};

struct SupervisoryUiModel {
    std::string mode_title;
    std::string state_label;
    std::string state_description;
    cv::Scalar state_color;
    std::string motion_label;
    cv::Scalar motion_color;
    std::string operator_prompt;
    std::string next_action;
    std::string freeze_reason;
    std::string footer_info;
    RuntimeLatencyMetrics latency;
    std::vector<double> vision_latency_history_us;
    std::vector<double> unsafe_detect_history_ms;
    std::vector<double> freeze_pipeline_history_ms;
    std::vector<double> freeze_cmd_history_ms;
    std::vector<double> total_stop_history_ms;
    std::vector<double> ack_resume_history_ms;
    std::vector<SupervisoryUiRow> status_rows;
    bool show_focus = false;
    std::string focus_label;
    cv::Scalar focus_color;
    double focus_fraction = 0.0;
    std::string camera_hud_text;
    cv::Scalar camera_hud_color;
    std::string camera_bottom_left;
    std::string camera_bottom_right;
    bool show_frozen_overlay = false;
    std::string frozen_overlay_title;
    std::string frozen_overlay_subtitle;
    bool show_waiting_overlay = false;
    std::string waiting_overlay_text;
    bool emphasise_danger = false;
};

cv::Scalar severityColor(const std::string& value) {
    if (value.find("FAULT") != std::string::npos ||
        value.find("FROZEN") != std::string::npos ||
        value.find("UNSAFE") != std::string::npos ||
        value.find("DETECTED") != std::string::npos ||
        value.find("OUTSIDE") != std::string::npos) {
        return cv::Scalar(60, 60, 200);
    }
    if (value.find("WAIT") != std::string::npos ||
        value.find("PENDING") != std::string::npos ||
        value.find("BLOCKED") != std::string::npos) {
        return cv::Scalar(50, 180, 230);
    }
    if (value.find("SAFE") != std::string::npos ||
        value.find("ALLOWED") != std::string::npos ||
        value.find("RUNNING") != std::string::npos ||
        value.find("READY") != std::string::npos) {
        return cv::Scalar(60, 170, 80);
    }
    return cv::Scalar(25, 25, 25);
}

void drawPanel(cv::Mat& frame,
               cv::Point top_left,
               cv::Point bottom_right,
               const cv::Scalar& fill,
               const cv::Scalar& border) {
    drawRectangle(frame, top_left, bottom_right, fill, -1);
    drawRectangle(frame, top_left, bottom_right, border, 1);
}

void drawSparkline(cv::Mat& frame,
                   cv::Point top_left,
                   cv::Point bottom_right,
                   const std::vector<double>& samples,
                   const cv::Scalar& line_color) {
    const cv::Scalar panel_fill(235, 235, 235);
    const cv::Scalar panel_border(220, 220, 220);
    drawPanel(frame, top_left, bottom_right, panel_fill, panel_border);

    const int inner_x1 = top_left.x + 2;
    const int inner_y1 = top_left.y + 2;
    const int inner_x2 = bottom_right.x - 2;
    const int inner_y2 = bottom_right.y - 2;
    if (inner_x2 <= inner_x1 || inner_y2 <= inner_y1 || samples.empty()) {
        return;
    }

    double min_value = samples.front();
    double max_value = samples.front();
    for (double sample : samples) {
        min_value = std::min(min_value, sample);
        max_value = std::max(max_value, sample);
    }
    if (max_value - min_value < 1e-6) {
        max_value = min_value + 1.0;
    }

    const int usable_width = std::max(1, inner_x2 - inner_x1);
    const std::size_t count = samples.size();
    for (std::size_t i = 1; i < count; ++i) {
        const int x_prev = inner_x1 +
                           static_cast<int>((usable_width * (i - 1)) /
                                            std::max<std::size_t>(1, count - 1));
        const int x_curr = inner_x1 +
                           static_cast<int>((usable_width * i) /
                                            std::max<std::size_t>(1, count - 1));
        const double prev_norm = (samples[i - 1] - min_value) / (max_value - min_value);
        const double curr_norm = (samples[i] - min_value) / (max_value - min_value);
        const int y_prev = inner_y2 -
                           static_cast<int>(prev_norm * (inner_y2 - inner_y1));
        const int y_curr = inner_y2 -
                           static_cast<int>(curr_norm * (inner_y2 - inner_y1));
        drawLine(frame,
                 cv::Point(x_prev, y_prev),
                 cv::Point(x_curr, y_curr),
                 line_color,
                 1);
    }
}

void drawSupervisoryGui(cv::Mat& frame, const SupervisoryUiModel& model) {
    if (frame.empty()) {
        return;
    }

    const int width = frameWidth(frame);
    const int height = frameHeight(frame);
    const int header_height = std::max(34, height / 14);
    const int state_bar_height = std::max(28, height / 16);
    const int left_width = std::max(160, width / 4);
    const int right_width = std::max(180, width / 4);
    const int panel_top = header_height + state_bar_height + 8;
    const int panel_bottom = height - 8;
    const int left_x2 = left_width;
    const int right_x1 = width - right_width;
    const int camera_x1 = left_x2 + 8;
    const int camera_x2 = right_x1 - 8;
    const int camera_y1 = panel_top;
    const int camera_y2 = panel_bottom;

    const cv::Scalar panel_fill(245, 245, 245);
    const cv::Scalar panel_border(220, 220, 220);
    const cv::Scalar primary_text(25, 25, 25);
    const cv::Scalar muted_text(120, 120, 120);
    const cv::Scalar info_color(170, 140, 60);
    const cv::Scalar white(255, 255, 255);

    drawPanel(frame,
              cv::Point(0, 0),
              cv::Point(width - 1, header_height),
              panel_fill,
              panel_border);
    drawPanel(frame,
              cv::Point(0, header_height),
              cv::Point(width - 1, header_height + state_bar_height),
              panel_fill,
              panel_border);
    drawPanel(frame,
              cv::Point(0, panel_top),
              cv::Point(left_x2, panel_bottom),
              panel_fill,
              panel_border);
    drawPanel(frame,
              cv::Point(right_x1, panel_top),
              cv::Point(width - 1, panel_bottom),
              panel_fill,
              panel_border);
    drawRectangle(frame,
                  cv::Point(camera_x1, camera_y1),
                  cv::Point(camera_x2, camera_y2),
                  panel_border,
                  1);

    drawPanel(frame,
              cv::Point(12, 6),
              cv::Point(40, 32),
              panel_fill,
              model.state_color);
    cv::putText(frame,
                "A",
                cv::Point(22, 25),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                primary_text,
                2);

    cv::putText(frame,
                "ARGUS",
                cv::Point(52, 21),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                primary_text,
                2);
    cv::putText(frame,
                "Safety Supervisor",
                cv::Point(52, 34),
                cv::FONT_HERSHEY_SIMPLEX,
                0.35,
                muted_text,
                1);
    cv::putText(frame,
                "ONLINE",
                cv::Point(width - 135, 20),
                uiPlainFontFace(),
                0.9,
                cv::Scalar(60, 170, 80),
                1);
    cv::putText(frame,
                model.mode_title,
                cv::Point(width - 135, 34),
                uiPlainFontFace(),
                0.9,
                primary_text,
                1);

    cv::putText(frame,
                model.state_label,
                cv::Point(16, header_height + 20),
                uiPlainFontFace(),
                1.0,
                model.state_color,
                1);
    cv::putText(frame,
                model.state_description,
                cv::Point(84, header_height + 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.38,
                muted_text,
                1);
    cv::putText(frame,
                "Operator -> " + model.operator_prompt,
                cv::Point(width - 240, header_height + 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.38,
                model.state_color,
                1);

    const int card_margin = 12;
    const int card_width = left_x2 - (2 * card_margin);
    int card_y = panel_top + 10;
    auto drawLeftCard = [&](int height_px,
                            const cv::Scalar& border_color,
                            const auto& painter) {
        drawPanel(frame,
                  cv::Point(card_margin, card_y),
                  cv::Point(card_margin + card_width, card_y + height_px),
                  white,
                  border_color);
        painter(card_margin, card_y);
        card_y += height_px + 10;
    };

    drawLeftCard(82, model.state_color, [&](int x, int y0) {
        cv::putText(frame,
                    "SAFETY STATE",
                    cv::Point(x + 12, y0 + 16),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.35,
                    muted_text,
                    1);
        cv::putText(frame,
                    model.state_label,
                    cv::Point(x + 12, y0 + 48),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.85,
                    model.state_color,
                    2);
        cv::putText(frame,
                    model.state_description,
                    cv::Point(x + 12, y0 + 66),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.34,
                    primary_text,
                    1);
    });

    drawLeftCard(62, model.motion_color, [&](int x, int y0) {
        cv::putText(frame,
                    "MOTION GATE",
                    cv::Point(x + 12, y0 + 16),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.35,
                    muted_text,
                    1);
        cv::putText(frame,
                    std::string("MOTION ") + model.motion_label,
                    cv::Point(x + 12, y0 + 44),
                    uiPlainFontFace(),
                    1.0,
                    model.motion_color,
                    1);
    });

    drawLeftCard(62, panel_border, [&](int x, int y0) {
        cv::putText(frame,
                    "NEXT ACTION",
                    cv::Point(x + 12, y0 + 16),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.35,
                    muted_text,
                    1);
        cv::putText(frame,
                    model.next_action,
                    cv::Point(x + 12, y0 + 44),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.42,
                    primary_text,
                    1);
    });

    if (model.freeze_reason != "NONE" && model.freeze_reason != "N/A") {
        drawLeftCard(62, cv::Scalar(60, 60, 200), [&](int x, int y0) {
            cv::putText(frame,
                        "FREEZE REASON",
                        cv::Point(x + 12, y0 + 16),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.35,
                        cv::Scalar(60, 60, 200),
                        1);
            cv::putText(frame,
                        model.freeze_reason,
                        cv::Point(x + 12, y0 + 44),
                        uiPlainFontFace(),
                        1.0,
                        severityColor(model.freeze_reason),
                        1);
        });
    }

    cv::putText(frame,
                model.camera_hud_text,
                cv::Point(camera_x1 + 12, camera_y1 + 20),
                uiPlainFontFace(),
                0.95,
                model.camera_hud_color,
                1);
    cv::putText(frame,
                "FOCUS " + model.focus_label,
                cv::Point(camera_x2 - 145, camera_y1 + 20),
                uiPlainFontFace(),
                0.9,
                model.focus_color,
                1);
    cv::putText(frame,
                model.camera_bottom_left,
                cv::Point(camera_x1 + 12, camera_y2 - 10),
                uiPlainFontFace(),
                0.8,
                muted_text,
                1);
    cv::putText(frame,
                model.camera_bottom_right,
                cv::Point(camera_x2 - 90, camera_y2 - 10),
                uiPlainFontFace(),
                0.8,
                muted_text,
                1);

    int rx = right_x1 + 12;
    int ry = panel_top + 18;
    auto drawRightSectionTitle = [&](const std::string& title) {
        cv::putText(frame,
                    title,
                    cv::Point(rx, ry),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.35,
                    muted_text,
                    1);
        ry += 12;
        drawLine(frame,
                 cv::Point(right_x1 + 10, ry),
                 cv::Point(width - 12, ry),
                 panel_border,
                 1);
        ry += 16;
    };

    drawRightSectionTitle("SUBSYSTEMS");

    for (const auto& row : model.status_rows) {
        cv::putText(frame,
                    row.label,
                    cv::Point(rx, ry),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.42,
                    muted_text,
                    1);
        ry += 14;
        cv::putText(frame,
                    row.value,
                    cv::Point(rx, ry),
                    uiPlainFontFace(),
                    1.0,
                    row.value_color,
                    1);
        ry += 14;
    }

    if (model.show_focus) {
        ry += 6;
        drawRightSectionTitle("FOCUS");
        drawPanel(frame,
                  cv::Point(rx, ry),
                  cv::Point(width - 24, ry + 16),
                  cv::Scalar(235, 235, 235),
                  panel_border);
        const int bar_width = std::max(
            0,
            static_cast<int>((width - right_x1 - 36) *
                             std::clamp(model.focus_fraction, 0.0, 1.0)));
        if (bar_width > 0) {
            drawRectangle(frame,
                          cv::Point(rx + 1, ry + 1),
                          cv::Point(rx + bar_width, ry + 15),
                          model.focus_color,
                          -1);
        }
        ry += 32;
        cv::putText(frame,
                    model.focus_label,
                    cv::Point(rx, ry),
                    uiPlainFontFace(),
                    1.0,
                    model.focus_color,
                    1);
        ry += 18;
    }

    ry += 6;
    drawRightSectionTitle("LATENCY");
    cv::putText(frame,
                "vision_us " + std::to_string(model.latency.vision_us),
                cv::Point(rx, ry),
                uiPlainFontFace(),
                1.0,
                primary_text,
                1);
    ry += 16;
    cv::putText(frame,
                "unsafe_ms " +
                    formatLatencyMilliseconds(model.latency.unsafe_detect_ms),
                cv::Point(rx, ry),
                uiPlainFontFace(),
                1.0,
                info_color,
                1);
    ry += 16;
    cv::putText(frame,
                "stop_ms " +
                    formatLatencyMilliseconds(model.latency.total_stop_ms),
                cv::Point(rx, ry),
                uiPlainFontFace(),
                1.0,
                severityColor(formatLatencyMilliseconds(model.latency.total_stop_ms)),
                1);
    ry += 16;
    cv::putText(frame,
                "freeze_cmd_ms " +
                    formatLatencyMilliseconds(model.latency.freeze_cmd_ms),
                cv::Point(rx, ry),
                uiPlainFontFace(),
                1.0,
                primary_text,
                1);
    ry += 16;
    cv::putText(frame,
                "ack_resume_ms " +
                    formatLatencyMilliseconds(model.latency.ack_to_resume_ms),
                cv::Point(rx, ry),
                uiPlainFontFace(),
                1.0,
                primary_text,
                1);

    if (model.show_frozen_overlay) {
        drawRectangle(frame,
                      cv::Point(camera_x1 + 2, camera_y1 + 2),
                      cv::Point(camera_x2 - 2, camera_y2 - 2),
                      cv::Scalar(60, 60, 200),
                      3);
        cv::putText(frame,
                    model.frozen_overlay_title,
                    cv::Point(camera_x1 + 80, camera_y1 + 110),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.1,
                    cv::Scalar(60, 60, 200),
                    3);
        cv::putText(frame,
                    model.frozen_overlay_subtitle,
                    cv::Point(camera_x1 + 70, camera_y1 + 140),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    cv::Scalar(60, 60, 200),
                    1);
    } else if (model.show_waiting_overlay) {
        drawPanel(frame,
                  cv::Point(camera_x1 + 80, camera_y2 - 60),
                  cv::Point(camera_x2 - 80, camera_y2 - 28),
                  white,
                  cv::Scalar(50, 180, 230));
        cv::putText(frame,
                    model.waiting_overlay_text,
                    cv::Point(camera_x1 + 92, camera_y2 - 38),
                    uiPlainFontFace(),
                    0.95,
                    cv::Scalar(50, 180, 230),
                    1);
    }

    cv::putText(frame,
                model.footer_info,
                cv::Point(16, height - 14),
                uiPlainFontFace(),
                0.9,
                muted_text,
                1);

    const cv::Scalar border_color =
        model.emphasise_danger ? cv::Scalar(60, 60, 200) : model.state_color;
    const int border_thickness = model.emphasise_danger ? 5 : 3;
    drawRectangle(frame,
                  cv::Point(2, 2),
                  cv::Point(width - 3, height - 3),
                  border_color,
                  border_thickness);
}

void drawCameraOverlay(cv::Mat& frame, const SupervisoryUiModel& model) {
    if (frame.empty()) {
        return;
    }

    const int width = frameWidth(frame);
    const int height = frameHeight(frame);
    const cv::Scalar muted_text(120, 120, 120);
    const cv::Scalar white(255, 255, 255);

    cv::putText(frame,
                model.camera_hud_text,
                cv::Point(14, 24),
                uiPlainFontFace(),
                0.95,
                model.camera_hud_color,
                1);
    if (model.show_focus) {
        const std::string focus_text = "FOCUS " + model.focus_label;
        const int focus_x = std::max(12, width - 290);
        cv::putText(frame,
                    focus_text,
                    cv::Point(focus_x, 24),
                    uiPlainFontFace(),
                    0.9,
                    model.focus_color,
                    1);
    }

    cv::putText(frame,
                model.camera_bottom_left,
                cv::Point(12, height - 12),
                uiPlainFontFace(),
                0.8,
                muted_text,
                1);
    cv::putText(frame,
                model.camera_bottom_right,
                cv::Point(std::max(12, width - 90), height - 12),
                uiPlainFontFace(),
                0.8,
                muted_text,
                1);

    if (model.show_frozen_overlay) {
        drawRectangle(frame,
                      cv::Point(6, 6),
                      cv::Point(width - 7, height - 7),
                      cv::Scalar(60, 60, 200),
                      3);
        const int title_y = std::max(80, height / 3);
        cv::putText(frame,
                    model.frozen_overlay_title,
                    cv::Point(24, title_y),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.1,
                    cv::Scalar(60, 60, 200),
                    3);
        cv::putText(frame,
                    model.frozen_overlay_subtitle,
                    cv::Point(24, title_y + 34),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cv::Scalar(60, 60, 200),
                    1);
    } else if (model.show_waiting_overlay) {
        const int box_w = std::max(320, width - 120);
        const int x1 = std::max(20, (width - box_w) / 2);
        const int y1 = std::max(40, height - 74);
        drawPanel(frame,
                  cv::Point(x1, y1),
                  cv::Point(x1 + box_w, y1 + 34),
                  white,
                  cv::Scalar(50, 180, 230));
        cv::putText(frame,
                    model.waiting_overlay_text,
                    cv::Point(x1 + 10, y1 + 22),
                    uiPlainFontFace(),
                    0.85,
                    cv::Scalar(50, 180, 230),
                    1);
    }

    const cv::Scalar border_color =
        model.emphasise_danger ? cv::Scalar(60, 60, 200) : model.state_color;
    const int border_thickness = model.emphasise_danger ? 4 : 2;
    drawRectangle(frame,
                  cv::Point(2, 2),
                  cv::Point(width - 3, height - 3),
                  border_color,
                  border_thickness);
}

void drawStatusDashboard(cv::Mat& frame, const SupervisoryUiModel& model) {
    if (frame.empty()) {
        return;
    }

    const int width = frameWidth(frame);
    const int height = frameHeight(frame);
    const int header_height = 52;
    const int state_bar_height = 32;
    const int body_top = header_height + state_bar_height + 8;
    const cv::Scalar panel_fill(245, 245, 245);
    const cv::Scalar panel_border(220, 220, 220);
    const cv::Scalar primary_text(25, 25, 25);
    const cv::Scalar muted_text(120, 120, 120);
    const cv::Scalar white(255, 255, 255);

    frame.setTo(panel_fill);
    drawPanel(frame,
              cv::Point(0, 0),
              cv::Point(width - 1, header_height),
              panel_fill,
              panel_border);
    drawPanel(frame,
              cv::Point(0, header_height),
              cv::Point(width - 1, header_height + state_bar_height),
              panel_fill,
              panel_border);

    drawPanel(frame,
              cv::Point(10, 8),
              cv::Point(38, 34),
              panel_fill,
              model.state_color);
    cv::putText(frame,
                "A",
                cv::Point(20, 27),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                primary_text,
                2);

    cv::putText(frame,
                "ARGUS",
                cv::Point(50, 23),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                primary_text,
                2);
    cv::putText(frame,
                "Safety Supervisor",
                cv::Point(50, 38),
                cv::FONT_HERSHEY_SIMPLEX,
                0.35,
                muted_text,
                1);
    cv::putText(frame,
                model.mode_title,
                cv::Point(width - 124, 31),
                uiPlainFontFace(),
                0.9,
                primary_text,
                1);

    cv::putText(frame,
                model.state_label,
                cv::Point(12, header_height + 22),
                uiPlainFontFace(),
                1.0,
                model.state_color,
                1);
    cv::putText(frame,
                model.state_description,
                cv::Point(92, header_height + 22),
                cv::FONT_HERSHEY_SIMPLEX,
                0.36,
                muted_text,
                1);
    cv::putText(frame,
                "Operator -> " + model.operator_prompt,
                cv::Point(12, header_height + 34),
                cv::FONT_HERSHEY_SIMPLEX,
                0.32,
                model.state_color,
                1);

    const int card_margin = 12;
    const int card_width = width - (2 * card_margin);
    int card_y = body_top;
    auto drawCard = [&](int height_px,
                        const cv::Scalar& border_color,
                        const auto& painter) {
        drawPanel(frame,
                  cv::Point(card_margin, card_y),
                  cv::Point(card_margin + card_width, card_y + height_px),
                  white,
                  border_color);
        painter(card_margin, card_y);
        card_y += height_px + 8;
    };

    drawCard(76, model.state_color, [&](int x, int y0) {
        cv::putText(frame,
                    "SAFETY STATE",
                    cv::Point(x + 10, y0 + 15),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.34,
                    muted_text,
                    1);
        cv::putText(frame,
                    model.state_label,
                    cv::Point(x + 10, y0 + 44),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    model.state_color,
                    2);
        cv::putText(frame,
                    model.state_description,
                    cv::Point(x + 10, y0 + 62),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.34,
                    primary_text,
                    1);
    });

    drawCard(56, model.motion_color, [&](int x, int y0) {
        cv::putText(frame,
                    "MOTION GATE",
                    cv::Point(x + 10, y0 + 15),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.34,
                    muted_text,
                    1);
        cv::putText(frame,
                    std::string("MOTION ") + model.motion_label,
                    cv::Point(x + 10, y0 + 40),
                    uiPlainFontFace(),
                    1.0,
                    model.motion_color,
                    1);
    });

    drawCard(56, panel_border, [&](int x, int y0) {
        cv::putText(frame,
                    "NEXT ACTION",
                    cv::Point(x + 10, y0 + 15),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.34,
                    muted_text,
                    1);
        cv::putText(frame,
                    model.next_action,
                    cv::Point(x + 10, y0 + 40),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.4,
                    primary_text,
                    1);
    });

    const bool freeze_reason_active =
        model.freeze_reason != "NONE" && model.freeze_reason != "N/A";
    drawCard(56,
             freeze_reason_active ? cv::Scalar(60, 60, 200) : panel_border,
             [&](int x, int y0) {
                 cv::putText(frame,
                             "FREEZE REASON",
                             cv::Point(x + 10, y0 + 15),
                             cv::FONT_HERSHEY_SIMPLEX,
                             0.34,
                             freeze_reason_active ? cv::Scalar(60, 60, 200) : muted_text,
                             1);
                 cv::putText(frame,
                             model.freeze_reason,
                             cv::Point(x + 10, y0 + 40),
                             uiPlainFontFace(),
                             1.0,
                             freeze_reason_active ? severityColor(model.freeze_reason)
                                                  : muted_text,
                             1);
             });

    int rx = card_margin + 2;
    int ry = card_y + 12;
    auto drawSectionTitle = [&](const std::string& title) {
        cv::putText(frame,
                    title,
                    cv::Point(rx, ry),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.34,
                    muted_text,
                    1);
        ry += 8;
        drawLine(frame,
                 cv::Point(card_margin, ry),
                 cv::Point(width - card_margin, ry),
                 panel_border,
                 1);
        ry += 14;
    };

    drawSectionTitle("SUBSYSTEMS");
    for (const auto& row : model.status_rows) {
        cv::putText(frame,
                    row.label,
                    cv::Point(rx, ry),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.4,
                    muted_text,
                    1);
        ry += 14;
        cv::putText(frame,
                    row.value,
                    cv::Point(rx, ry),
                    uiPlainFontFace(),
                    1.0,
                    row.value_color,
                    1);
        ry += 14;
    }

    const cv::Scalar border_color =
        model.emphasise_danger ? cv::Scalar(60, 60, 200) : model.state_color;
    drawRectangle(frame,
                  cv::Point(2, 2),
                  cv::Point(width - 3, height - 3),
                  border_color,
                  model.emphasise_danger ? 4 : 2);
}

void drawMetricsDashboard(cv::Mat& frame, const SupervisoryUiModel& model) {
    if (frame.empty()) {
        return;
    }

    const int width = frameWidth(frame);
    const int header_height = 52;
    const int state_bar_height = 32;
    const int body_top = header_height + state_bar_height + 8;
    const cv::Scalar panel_fill(245, 245, 245);
    const cv::Scalar panel_border(220, 220, 220);
    const cv::Scalar primary_text(25, 25, 25);
    const cv::Scalar muted_text(120, 120, 120);
    const cv::Scalar info_color(170, 140, 60);
    const cv::Scalar white(255, 255, 255);

    frame.setTo(panel_fill);
    drawPanel(frame,
              cv::Point(0, 0),
              cv::Point(width - 1, header_height),
              panel_fill,
              panel_border);
    drawPanel(frame,
              cv::Point(0, header_height),
              cv::Point(width - 1, header_height + state_bar_height),
              panel_fill,
              panel_border);

    cv::putText(frame,
                "ARGUS",
                cv::Point(12, 23),
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                primary_text,
                2);
    cv::putText(frame,
                "Metrics",
                cv::Point(12, 38),
                cv::FONT_HERSHEY_SIMPLEX,
                0.35,
                muted_text,
                1);
    cv::putText(frame,
                model.mode_title,
                cv::Point(width - 124, 31),
                uiPlainFontFace(),
                0.9,
                primary_text,
                1);

    cv::putText(frame,
                "FOCUS + LATENCY",
                cv::Point(12, header_height + 22),
                uiPlainFontFace(),
                1.0,
                model.state_color,
                1);
    cv::putText(frame,
                "Event metrics and trend",
                cv::Point(12, header_height + 34),
                cv::FONT_HERSHEY_SIMPLEX,
                0.32,
                muted_text,
                1);

    const int card_margin = 12;
    const int card_width = width - (2 * card_margin);
    int y = body_top;

    drawPanel(frame,
              cv::Point(card_margin, y),
              cv::Point(card_margin + card_width, y + 62),
              white,
              panel_border);
    cv::putText(frame,
                "FOCUS",
                cv::Point(card_margin + 10, y + 15),
                cv::FONT_HERSHEY_SIMPLEX,
                0.34,
                muted_text,
                1);
    drawPanel(frame,
              cv::Point(card_margin + 10, y + 22),
              cv::Point(width - 22, y + 38),
              cv::Scalar(235, 235, 235),
              panel_border);
    const int bar_width = std::max(
        0,
        static_cast<int>((width - card_margin - 34) *
                         std::clamp(model.focus_fraction, 0.0, 1.0)));
    if (model.show_focus && bar_width > 0) {
        drawRectangle(frame,
                      cv::Point(card_margin + 11, y + 23),
                      cv::Point(card_margin + 11 + bar_width, y + 37),
                      model.focus_color,
                      -1);
    }
    cv::putText(frame,
                model.show_focus ? model.focus_label : "N/A",
                cv::Point(card_margin + 10, y + 56),
                uiPlainFontFace(),
                0.95,
                model.show_focus ? model.focus_color : muted_text,
                1);
    y += 74;

    cv::putText(frame,
                "LATENCY",
                cv::Point(card_margin + 2, y + 8),
                cv::FONT_HERSHEY_SIMPLEX,
                0.34,
                muted_text,
                1);
    y += 16;
    drawLine(frame,
             cv::Point(card_margin, y),
             cv::Point(width - card_margin, y),
             panel_border,
             1);
    y += 14;

    auto drawLatencyRow = [&](const std::string& label,
                              const std::string& value,
                              const std::vector<double>& history,
                              const cv::Scalar& color) {
        cv::putText(frame,
                    label + " " + value,
                    cv::Point(card_margin + 2, y),
                    uiPlainFontFace(),
                    0.95,
                    color,
                    1);
        drawSparkline(frame,
                      cv::Point(card_margin + 2, y + 4),
                      cv::Point(width - 20, y + 20),
                      history,
                      color);
        y += 28;
    };

    drawLatencyRow("vision_us",
                   std::to_string(model.latency.vision_us),
                   model.vision_latency_history_us,
                   primary_text);
    drawLatencyRow("unsafe_ms",
                   formatLatencyMilliseconds(model.latency.unsafe_detect_ms),
                   model.unsafe_detect_history_ms,
                   info_color);
    drawLatencyRow("freeze_pipeline_ms",
                   formatLatencyMilliseconds(model.latency.freeze_pipeline_ms),
                   model.freeze_pipeline_history_ms,
                   cv::Scalar(90, 140, 220));
    drawLatencyRow("freeze_cmd_ms",
                   formatLatencyMilliseconds(model.latency.freeze_cmd_ms),
                   model.freeze_cmd_history_ms,
                   primary_text);
    drawLatencyRow("stop_ms",
                   formatLatencyMilliseconds(model.latency.total_stop_ms),
                   model.total_stop_history_ms,
                   severityColor(formatLatencyMilliseconds(model.latency.total_stop_ms)));
    drawLatencyRow("ack_resume_ms",
                   formatLatencyMilliseconds(model.latency.ack_to_resume_ms),
                   model.ack_resume_history_ms,
                   cv::Scalar(120, 100, 180));

    const cv::Scalar border_color =
        model.emphasise_danger ? cv::Scalar(60, 60, 200) : model.state_color;
    drawRectangle(frame,
                  cv::Point(2, 2),
                  cv::Point(width - 3, frameHeight(frame) - 3),
                  border_color,
                  model.emphasise_danger ? 4 : 2);
}

const char* safetyStateToString(SafetyState state) {
    switch (state) {
        case SafetyState::SAFE:
            return "SAFE";
        case SafetyState::TOOL_NOT_DETECTED:
            return "TOOL_NOT_DETECTED";
        case SafetyState::OUTSIDE_ALLOWED_ZONE:
            return "OUTSIDE_ALLOWED_ZONE";
        case SafetyState::EXCESSIVE_SPEED:
            return "EXCESSIVE_SPEED";
        case SafetyState::INVALID_ORIENTATION:
            return "INVALID_ORIENTATION";
        default:
            return "UNKNOWN";
    }
}

FreezeReason mapSafetyToFreezeReason(SafetyState state) {
    switch (state) {
        case SafetyState::TOOL_NOT_DETECTED:
            return FreezeReason::MARKER_LOST;
        case SafetyState::OUTSIDE_ALLOWED_ZONE:
            return FreezeReason::MARKER_OUT_OF_ROI;
        case SafetyState::EXCESSIVE_SPEED:
        case SafetyState::INVALID_ORIENTATION:
            return FreezeReason::POSITION_ERROR;
        case SafetyState::SAFE:
        default:
            return FreezeReason::UNKNOWN_FAULT;
    }
}

const char* freezeReasonToString(FreezeReason reason) {
    switch (reason) {
        case FreezeReason::NONE:
            return "NONE";
        case FreezeReason::MARKER_LOST:
            return "MARKER_LOST";
        case FreezeReason::MARKER_OUT_OF_ROI:
            return "MARKER_OUT_OF_ROI";
        case FreezeReason::VISION_TIMEOUT:
            return "VISION_TIMEOUT";
        case FreezeReason::POSITION_ERROR:
            return "POSITION_ERROR";
        case FreezeReason::WATCHDOG_TIMEOUT:
            return "WATCHDOG_TIMEOUT";
        case FreezeReason::UNKNOWN_FAULT:
        default:
            return "UNKNOWN_FAULT";
    }
}

const char* cameraBackendPreferenceToString(
    CameraCapture::BackendPreference preference) {
    switch (preference) {
        case CameraCapture::BackendPreference::Auto:
            return "auto";
        case CameraCapture::BackendPreference::OpenCvVideoCapture:
            return "opencv";
        case CameraCapture::BackendPreference::Libcamera2OpenCv:
            return "libcamera2opencv";
    }
    return "unknown";
}

const char* interlockStateToString(InterlockState state) {
    switch (state) {
        case InterlockState::SAFE:
            return "SAFE";
        case InterlockState::FROZEN:
            return "FROZEN";
        case InterlockState::FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

const char* motionControllerStateToString(MotionOutputState state) {
    switch (state) {
        case MotionOutputState::UNINITIALISED:
            return "UNINITIALISED";
        case MotionOutputState::DISABLED:
            return "DISABLED";
        case MotionOutputState::ENABLED:
            return "ENABLED";
        case MotionOutputState::FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

struct SmokeJointOffsets {
    int base{0};
    int lower{0};
    int upper{0};
    int grip{0};
};

struct SmokeJointSpec {
    const char* logical_name;
    const char* mearm_name;
    const char* visual_check;
    std::uint8_t channel;
    int min_offset;
    int max_offset;
};

struct SmokeJointRunPlan {
    std::array<std::size_t, MotionController::kServoCount> indices{};
    std::size_t count{0};
};

struct JointPulseCalibration {
    std::uint16_t neg90_ticks;
    std::uint16_t zero_ticks;
    std::uint16_t pos90_ticks;
};

struct JointCalibrationMarks {
    bool has_neg90{false};
    bool has_zero{false};
    bool has_pos90{false};
    std::uint16_t neg90_ticks{0};
    std::uint16_t zero_ticks{0};
    std::uint16_t pos90_ticks{0};
};

constexpr std::array<SmokeJointSpec, MotionController::kServoCount> kSmokeJointSpecs = {{
    {"base", "MeArm BASE", "yaw left/right", kMotionChannelMap.base, kSmokeBaseMinOffset, kSmokeBaseMaxOffset},
    {"lower", "MeArm LEFT", "raise/lower", kMotionChannelMap.lower, kSmokeLowerMinOffset, kSmokeLowerMaxOffset},
    {"upper", "MeArm RIGHT", "bend/extend", kMotionChannelMap.upper, kSmokeUpperMinOffset, kSmokeUpperMaxOffset},
    {"grip", "MeArm CLAW", "open/close", kMotionChannelMap.gripper, kSmokeGripMinOffset, kSmokeGripMaxOffset},
}};

constexpr std::array<JointPulseCalibration, MotionController::kServoCount>
    kJointPulseCalibration = {{
        {100, 300, 500},
        {100, 300, 500},
        {100, 290, 500},
        {100, 300, 500},
    }};

constexpr SmokeJointOffsets kSmokeHomePose{0, 0, 0, 0};
constexpr SmokeJointOffsets kSurgeryRetractPose{0, 35, -30, 0};

struct DemoPoseStep {
    const char* name;
    SmokeJointOffsets offsets;
};

struct LiveRoutineDefinition {
    int number;
    const char* name;
    const DemoPoseStep* steps;
    std::size_t step_count;
    bool auto_progress;
};

constexpr DemoPoseStep kDemoHomeStep{"HOME", kSmokeHomePose};
constexpr DemoPoseStep kSurgeryRetractStep{"RETRACT_SAFE", kSurgeryRetractPose};

constexpr std::array<DemoPoseStep, 11> kLiveSurgeryCutSequence = {{
    {"GRIP +90 (TOOL)", {0, 0, 0, kSurgeryGripHoldOffset}},
    {"CUT P1 FORWARD", {0, 0, kSurgeryForwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P1 DOWN", {0, kSurgeryPassOneDepthOffset, kSurgeryForwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P1 BACKWARD", {0, kSurgeryPassOneDepthOffset, kSurgeryBackwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P2 FORWARD", {0, 0, kSurgeryForwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P2 DOWN (DEEPER)", {0, kSurgeryPassTwoDepthOffset, kSurgeryForwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P2 BACKWARD", {0, kSurgeryPassTwoDepthOffset, kSurgeryBackwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P3 FORWARD", {0, 0, kSurgeryForwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P3 DOWN (FAILURE PASS)", {0, kSurgeryPassThreeDepthOffset, kSurgeryForwardOffset, kSurgeryGripHoldOffset}},
    {"CUT P3 BACKWARD", {0, kSurgeryPassThreeDepthOffset, kSurgeryBackwardOffset, kSurgeryGripHoldOffset}},
    {"HOME", {0, 0, 0, kSurgeryGripHoldOffset}},
}};

constexpr std::array<DemoPoseStep, 5> kLiveBaseScanSequence = {{
    {"HOME", {0, 0, 0, 0}},
    {"BASE +45", {kDemoBaseStep, 0, 0, 0}},
    {"HOME", {0, 0, 0, 0}},
    {"BASE -45", {-kDemoBaseStep, 0, 0, 0}},
    {"HOME", {0, 0, 0, 0}},
}};

constexpr std::array<DemoPoseStep, 5> kLiveGripPulseSequence = {{
    {"HOME", {0, 0, 0, 0}},
    {"GRIP +45", {0, 0, 0, kDemoGripStep}},
    {"HOME", {0, 0, 0, 0}},
    {"GRIP -45", {0, 0, 0, -kDemoGripStep}},
    {"HOME", {0, 0, 0, 0}},
}};

LiveRoutineDefinition getLiveRoutineDefinition(std::size_t index) {
    switch (index) {
        case 1:
            return {1,
                    "SURGERY_CUT",
                    kLiveSurgeryCutSequence.data(),
                    kLiveSurgeryCutSequence.size(),
                    true};
        case 2:
            return {2,
                    "BASE_SCAN",
                    kLiveBaseScanSequence.data(),
                    kLiveBaseScanSequence.size(),
                    true};
        case 3:
            return {3,
                    "GRIP_PULSE",
                    kLiveGripPulseSequence.data(),
                    kLiveGripPulseSequence.size(),
                    true};
        case 0:
        default:
            return {0, "MANUAL", nullptr, 0, false};
    }
}

bool liveRoutineIndexFromKey(int key, std::size_t& index) {
    switch (key) {
        case '0':
            index = 0;
            return true;
        case '1':
            index = 1;
            return true;
        case '2':
            index = 2;
            return true;
        case '3':
            index = 3;
            return true;
        default:
            return false;
    }
}

std::string toLowerCopy(std::string text);

const char* smokeJointSelectionToString(AppController::SmokeJoint joint) {
    switch (joint) {
        case AppController::SmokeJoint::All:
            return "all";
        case AppController::SmokeJoint::Base:
            return "base";
        case AppController::SmokeJoint::Lower:
            return "lower";
        case AppController::SmokeJoint::Upper:
            return "upper";
        case AppController::SmokeJoint::Grip:
            return "grip";
        default:
            return "unknown";
    }
}

const char* smokeJointIndexToString(std::size_t index) {
    switch (index) {
        case 0:
            return "base";
        case 1:
            return "lower";
        case 2:
            return "upper";
        case 3:
            return "grip";
        default:
            return "unknown";
    }
}

bool smokeJointIndexFromName(const std::string& name, std::size_t& index) {
    const std::string lower = toLowerCopy(name);
    if (lower == "base") {
        index = 0;
        return true;
    }
    if (lower == "lower") {
        index = 1;
        return true;
    }
    if (lower == "upper") {
        index = 2;
        return true;
    }
    if (lower == "grip" || lower == "gripper") {
        index = 3;
        return true;
    }
    return false;
}

SmokeJointRunPlan makeSmokeJointRunPlan(AppController::SmokeJoint joint) {
    SmokeJointRunPlan plan{};

    switch (joint) {
        case AppController::SmokeJoint::Base:
            plan.indices[0] = 0;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::Lower:
            plan.indices[0] = 1;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::Upper:
            plan.indices[0] = 2;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::Grip:
            plan.indices[0] = 3;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::All:
        default:
            plan.indices[0] = 0;
            plan.indices[1] = 1;
            plan.indices[2] = 2;
            plan.indices[3] = 3;
            plan.count = 4;
            break;
    }

    return plan;
}

int clampOffsetValue(int value, int min_value, int max_value, bool& clamped) {
    const int bounded = std::clamp(value, min_value, max_value);
    if (bounded != value) {
        clamped = true;
    }
    return bounded;
}

SmokeJointOffsets clampOffsetsToWindow(const SmokeJointOffsets& requested,
                                       int base_min,
                                       int base_max,
                                       int lower_min,
                                       int lower_max,
                                       int upper_min,
                                       int upper_max,
                                       int grip_min,
                                       int grip_max,
                                       bool& clamped) {
    SmokeJointOffsets bounded = requested;
    bounded.base = clampOffsetValue(requested.base, base_min, base_max, clamped);
    bounded.lower = clampOffsetValue(requested.lower, lower_min, lower_max, clamped);
    bounded.upper = clampOffsetValue(requested.upper, upper_min, upper_max, clamped);
    bounded.grip = clampOffsetValue(requested.grip, grip_min, grip_max, clamped);
    return bounded;
}

SmokeJointOffsets clampSmokeOffsets(const SmokeJointOffsets& requested,
                                    bool& clamped) {
    return clampOffsetsToWindow(requested,
                                kSmokeBaseMinOffset,
                                kSmokeBaseMaxOffset,
                                kSmokeLowerMinOffset,
                                kSmokeLowerMaxOffset,
                                kSmokeUpperMinOffset,
                                kSmokeUpperMaxOffset,
                                kSmokeGripMinOffset,
                                kSmokeGripMaxOffset,
                                clamped);
}

SmokeJointOffsets clampDemoOffsets(const SmokeJointOffsets& requested,
                                   bool& clamped) {
    return clampOffsetsToWindow(requested,
                                kDemoBaseMinOffset,
                                kDemoBaseMaxOffset,
                                kDemoLowerMinOffset,
                                kDemoLowerMaxOffset,
                                kDemoUpperMinOffset,
                                kDemoUpperMaxOffset,
                                kDemoGripMinOffset,
                                kDemoGripMaxOffset,
                                clamped);
}

std::uint16_t logicalAngleToPulseTicks(std::size_t joint_index,
                                       int angle_degrees,
                                       bool& clamped) {
    const JointPulseCalibration& calibration =
        kJointPulseCalibration.at(joint_index);
    const int zero_ticks = static_cast<int>(calibration.zero_ticks);
    const int delta_ticks = angle_degrees >= 0
                                ? static_cast<int>(calibration.pos90_ticks) -
                                      zero_ticks
                                : zero_ticks -
                                      static_cast<int>(calibration.neg90_ticks);
    const int raw = zero_ticks +
                    static_cast<int>(std::lround(
                        static_cast<double>(delta_ticks) *
                        (static_cast<double>(angle_degrees) / 90.0)));
    const int bounded = std::clamp(
        raw, 0, static_cast<int>(MotionController::kMaxPulseTicks));
    if (bounded != raw) {
        clamped = true;
    }
    return static_cast<std::uint16_t>(bounded);
}

std::uint16_t clampPulseTicks(int raw_ticks, bool& clamped) {
    const int bounded = std::clamp(raw_ticks,
                                   0,
                                   static_cast<int>(MotionController::kMaxPulseTicks));
    if (bounded != raw_ticks) {
        clamped = true;
    }
    return static_cast<std::uint16_t>(bounded);
}

MeArmJointTargets makeSmokeTargets(const SmokeJointOffsets& requested,
                                   bool& clamped) {
    const SmokeJointOffsets bounded = clampSmokeOffsets(requested, clamped);
    return {
        logicalAngleToPulseTicks(0, bounded.base, clamped),
        logicalAngleToPulseTicks(1, bounded.lower, clamped),
        logicalAngleToPulseTicks(2, bounded.upper, clamped),
        logicalAngleToPulseTicks(3, bounded.grip, clamped)};
}

MeArmJointTargets makeDemoTargets(const SmokeJointOffsets& requested,
                                  bool& clamped) {
    const SmokeJointOffsets bounded = clampDemoOffsets(requested, clamped);
    return {
        logicalAngleToPulseTicks(0, bounded.base, clamped),
        logicalAngleToPulseTicks(1, bounded.lower, clamped),
        logicalAngleToPulseTicks(2, bounded.upper, clamped),
        logicalAngleToPulseTicks(3, bounded.grip, clamped)};
}

std::string formatOffsets(const SmokeJointOffsets& offsets) {
    std::ostringstream oss;
    oss << "{base=" << offsets.base
        << ", lower=" << offsets.lower
        << ", upper=" << offsets.upper
        << ", grip=" << offsets.grip
        << "}";
    return oss.str();
}

std::string formatTargets(const MeArmJointTargets& targets) {
    std::ostringstream oss;
    oss << "{base=" << targets.base_ticks
        << ", lower=" << targets.lower_ticks
        << ", upper=" << targets.upper_ticks
        << ", gripper=" << targets.gripper_ticks
        << "}";
    return oss.str();
}

std::uint16_t pulseForJoint(const MeArmJointTargets& targets, std::size_t index) {
    switch (index) {
        case 0:
            return targets.base_ticks;
        case 1:
            return targets.lower_ticks;
        case 2:
            return targets.upper_ticks;
        case 3:
            return targets.gripper_ticks;
        default:
            return 0;
    }
}

void setPulseForJoint(MeArmJointTargets& targets,
                      std::size_t index,
                      std::uint16_t ticks) {
    switch (index) {
        case 0:
            targets.base_ticks = ticks;
            break;
        case 1:
            targets.lower_ticks = ticks;
            break;
        case 2:
            targets.upper_ticks = ticks;
            break;
        case 3:
            targets.gripper_ticks = ticks;
            break;
        default:
            break;
    }
}

bool parseCalibrationPoint(const std::string& text,
                           const char*& label,
                           int& nominal_degrees) {
    const std::string lower = toLowerCopy(text);
    if (lower == "-90" || lower == "neg90" || lower == "minus90") {
        label = "-90";
        nominal_degrees = -90;
        return true;
    }
    if (lower == "0" || lower == "zero" || lower == "center" || lower == "centre") {
        label = "0";
        nominal_degrees = 0;
        return true;
    }
    if (lower == "+90" || lower == "90" || lower == "pos90" || lower == "plus90") {
        label = "+90";
        nominal_degrees = 90;
        return true;
    }
    return false;
}

std::string formatCalibrationMarks(std::size_t joint_index,
                                   const JointCalibrationMarks& marks) {
    std::ostringstream oss;
    oss << smokeJointIndexToString(joint_index) << ": "
        << "-90=";
    if (marks.has_neg90) {
        oss << marks.neg90_ticks;
    } else {
        oss << "unset";
    }
    oss << "  0=";
    if (marks.has_zero) {
        oss << marks.zero_ticks;
    } else {
        oss << "unset";
    }
    oss << "  +90=";
    if (marks.has_pos90) {
        oss << marks.pos90_ticks;
    } else {
        oss << "unset";
    }
    return oss.str();
}

std::string buildCalibrationSummary(
    const std::array<JointCalibrationMarks, MotionController::kServoCount>& marks) {
    std::ostringstream oss;
    oss << "# ARGUS servo calibration summary\n"
        << "# This report is informational only; values are not applied automatically.\n"
        << "# Format: joint  -90=<ticks>  0=<ticks>  +90=<ticks>\n";
    for (std::size_t i = 0; i < marks.size(); ++i) {
        oss << formatCalibrationMarks(i, marks[i]) << "\n";
    }
    return oss.str();
}

std::string toLowerCopy(std::string text) {
    std::transform(text.begin(),
                   text.end(),
                   text.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    return text;
}

}  // namespace

AppController::AppController() noexcept = default;

AppController::~AppController() noexcept = default;

int AppController::runGuardianScenarioDemo() {
    GuardianStateMachine guardian(2, 3);
    guardian.setOnFreezeCallback([]() {
        std::cout << ">>> ROBOTIC ARM: Emergency stop activated! <<<" << std::endl;
    });
    guardian.setOnClearFreezeCallback([]() {
        std::cout << ">>> ROBOTIC ARM: Motion resumed, system operational <<<"
                  << std::endl;
    });
    guardian.setOnStateChangeCallback([](GuardianState, GuardianState) {
        std::cout << ">>> STATE CHANGE NOTIFICATION: System transitioned <<<"
                  << std::endl;
    });

    std::cout << "\n========== GUARDIAN STATE MACHINE TEST ==========\n"
              << std::endl;

    std::cout << "Scenario 1: Normal Operation" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "Scenario 2: Single Bad Frame" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "Scenario 3: freezeCount Consecutive Bad Frames (Freeze)"
              << std::endl;
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.printStatus();

    std::cout << "Scenario 4: Frames During Frozen State" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "Scenario 5: Operator Acknowledgment/Reset" << std::endl;
    guardian.operatorAcknowledge();
    guardian.printStatus();

    std::cout << "Scenario 6: Bad Frame During Reset" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.printStatus();

    std::cout << "Scenario 7: recoverCount Consecutive Good Frames (Clear Freeze)"
              << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "========== TEST COMPLETE ==========\n" << std::endl;
    return 0;
}

int AppController::runButtonTest() {
    PhysicalButtonModule button_module;
    std::cout << "[BUTTON_TEST] physical button test\n"
              << "[BUTTON_TEST] press Ctrl+C to stop\n"
              << "[BUTTON_TEST] module: "
              << (button_module.available() ? "configured" : "disabled");
    if (button_module.available()) {
        std::cout << " (" << button_module.statusString() << ")";
    } else {
        std::cout << " (" << button_module.lastErrorString() << ")";
    }
    std::cout << std::endl;

    if (!button_module.available()) {
        return 1;
    }

    bool pressed = false;
    if (!button_module.readAcknowledgePressed(pressed)) {
        std::cerr << "[BUTTON_TEST] initial read failed: "
                  << button_module.lastErrorString() << std::endl;
        return 1;
    }

    bool last_pressed = pressed;
    std::cout << "[BUTTON_TEST] state="
              << (pressed ? "PRESSED" : "RELEASED") << std::endl;

    while (true) {
        if (!button_module.readAcknowledgePressed(pressed)) {
            std::cerr << "[BUTTON_TEST] read failed: "
                      << button_module.lastErrorString() << std::endl;
            return 1;
        }

        if (pressed != last_pressed) {
            last_pressed = pressed;
            std::cout << "[BUTTON_TEST] state="
                      << (pressed ? "PRESSED" : "RELEASED") << std::endl;
        }

        PhysicalButtonEvent event;
        while (button_module.poll(event)) {
            std::cout << "[BUTTON_TEST] event="
                      << PhysicalButtonModule::eventToString(event) << std::endl;
        }

        std::string timer_error;
        if (!waitForCppTimerDelay(std::chrono::milliseconds(20), timer_error)) {
            std::cerr << "[BUTTON_TEST] delay timer failed: " << timer_error
                      << std::endl;
            return 1;
        }
    }
}

int AppController::runServoCalibration() {
    std::cout
        << "[CAL] servo calibration console\n"
        << "[CAL] raw PCA9685 pulse control in ticks\n"
        << "[CAL] commands: base <ticks>, lower <ticks>, upper <ticks>, grip <ticks>\n"
        << "[CAL] nudges: base +5, base -5, grip +10, ...\n"
        << "[CAL] mark: mark <joint> <-90|0|+90>\n"
        << "[CAL] extras: home, status, summary, write [path], help\n"
        << "[CAL] quit: Ctrl+C or type 'exit'\n";

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       kMotionChannelMap)) {
        std::cerr << "[CAL] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[CAL] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    bool home_clamped = false;
    MeArmJointTargets current_targets =
        makeSmokeTargets(kSmokeHomePose, home_clamped);

    if (!motion_controller_.setTargets(current_targets)) {
        return fail(std::string("failed to stage home pulse set: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::array<JointCalibrationMarks, MotionController::kServoCount> marks{};
    std::cout << "[CAL] home " << formatTargets(current_targets) << std::endl;

    const auto previous_sigint = std::signal(SIGINT, handleInteractiveServoSignal);
    const auto previous_sigterm = std::signal(SIGTERM, handleInteractiveServoSignal);
    g_interactive_servo_stop_requested = 0;

    auto restoreSignals = [&]() {
        std::signal(SIGINT, previous_sigint);
        std::signal(SIGTERM, previous_sigterm);
    };

    auto applyTargets = [&](const MeArmJointTargets& requested,
                            const std::string& label) -> bool {
        MeArmJointTargets bounded = requested;
        bool clamped = false;
        bounded.base_ticks = clampPulseTicks(static_cast<int>(requested.base_ticks), clamped);
        bounded.lower_ticks = clampPulseTicks(static_cast<int>(requested.lower_ticks), clamped);
        bounded.upper_ticks = clampPulseTicks(static_cast<int>(requested.upper_ticks), clamped);
        bounded.gripper_ticks = clampPulseTicks(static_cast<int>(requested.gripper_ticks), clamped);

        if (!motion_controller_.setTargets(bounded)) {
            (void)fail(std::string("failed to set ") + label + ": " +
                       motion_controller_.lastErrorString());
            return false;
        }

        current_targets = bounded;
        std::cout << "[CAL] " << label << " " << formatTargets(current_targets);
        if (clamped) {
            std::cout << " [clamped]";
        }
        std::cout << std::endl;
        return true;
    };

    auto writeSummaryToFile = [&](const std::string& path) -> bool {
        std::ofstream out(path);
        if (!out.is_open()) {
            std::cerr << "[CAL] write failed: " << path << std::endl;
            return false;
        }
        out << buildCalibrationSummary(marks);
        out.close();
        std::cout << "[CAL] wrote " << path << std::endl;
        return true;
    };

    while (!g_interactive_servo_stop_requested) {
        std::cout << "cal> " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) {
            if (g_interactive_servo_stop_requested || std::cin.eof()) {
                break;
            }
            restoreSignals();
            return fail("stdin read failed");
        }

        std::istringstream iss(line);
        std::string command;
        if (!(iss >> command)) {
            continue;
        }

        command = toLowerCopy(command);
        if (command == "exit" || command == "quit") {
            break;
        }

        if (command == "help") {
            std::cout
                << "[CAL] commands: <joint> <ticks>, <joint> +/-<delta>, mark <joint> <-90|0|+90>, home, status, summary, write [path], exit"
                << std::endl;
            continue;
        }

        if (command == "home") {
            bool clamped = false;
            const MeArmJointTargets home_targets =
                makeSmokeTargets(kSmokeHomePose, clamped);
            if (!applyTargets(home_targets, "home")) {
                restoreSignals();
                return 1;
            }
            continue;
        }

        if (command == "status") {
            std::cout << "[CAL] " << formatTargets(current_targets) << std::endl;
            for (std::size_t i = 0; i < marks.size(); ++i) {
                std::cout << "[CAL] " << formatCalibrationMarks(i, marks[i])
                          << std::endl;
            }
            continue;
        }

        if (command == "summary") {
            std::cout << buildCalibrationSummary(marks);
            continue;
        }

        if (command == "write") {
            std::string path;
            if (!(iss >> path)) {
                path = "config/servo_calibration_latest.txt";
            }
            (void)writeSummaryToFile(path);
            continue;
        }

        if (command == "mark") {
            std::string joint_name;
            std::string point_name;
            if (!(iss >> joint_name >> point_name)) {
                std::cout << "[CAL] expected: mark <joint> <-90|0|+90>" << std::endl;
                continue;
            }

            std::size_t joint_index = 0;
            if (!smokeJointIndexFromName(joint_name, joint_index)) {
                std::cout << "[CAL] unknown joint: " << joint_name << std::endl;
                continue;
            }

            const char* point_label = nullptr;
            int nominal_degrees = 0;
            if (!parseCalibrationPoint(point_name, point_label, nominal_degrees)) {
                std::cout << "[CAL] unknown calibration point: " << point_name
                          << std::endl;
                continue;
            }

            JointCalibrationMarks& joint_marks = marks[joint_index];
            const std::uint16_t current_ticks = pulseForJoint(current_targets, joint_index);
            if (nominal_degrees < 0) {
                joint_marks.has_neg90 = true;
                joint_marks.neg90_ticks = current_ticks;
            } else if (nominal_degrees > 0) {
                joint_marks.has_pos90 = true;
                joint_marks.pos90_ticks = current_ticks;
            } else {
                joint_marks.has_zero = true;
                joint_marks.zero_ticks = current_ticks;
            }

            std::cout << "[CAL] marked " << smokeJointIndexToString(joint_index)
                      << " " << point_label << "=" << current_ticks << std::endl;
            continue;
        }

        std::size_t joint_index = 0;
        if (!smokeJointIndexFromName(command, joint_index)) {
            std::cout << "[CAL] unknown command or joint: " << command << std::endl;
            continue;
        }

        std::string value_text;
        if (!(iss >> value_text)) {
            std::cout << "[CAL] expected: <joint> <ticks>" << std::endl;
            continue;
        }

        int raw_value = 0;
        try {
            std::size_t parsed = 0;
            raw_value = std::stoi(value_text, &parsed);
            if (parsed != value_text.size()) {
                throw std::invalid_argument("trailing characters");
            }
        } catch (const std::exception&) {
            std::cout << "[CAL] invalid tick value: " << value_text << std::endl;
            continue;
        }

        MeArmJointTargets requested = current_targets;
        const int requested_ticks =
            static_cast<int>(pulseForJoint(current_targets, joint_index)) + 0;
        bool value_clamped = false;
        if (!value_text.empty() && (value_text[0] == '+' || value_text[0] == '-')) {
            setPulseForJoint(requested,
                             joint_index,
                             clampPulseTicks(requested_ticks + raw_value,
                                             value_clamped));
        } else {
            setPulseForJoint(requested,
                             joint_index,
                             clampPulseTicks(raw_value, value_clamped));
        }

        std::ostringstream label;
        label << smokeJointIndexToString(joint_index) << "="
              << pulseForJoint(requested, joint_index);
        if (!applyTargets(requested, label.str())) {
            restoreSignals();
            return 1;
        }
    }

    restoreSignals();
    std::cout << buildCalibrationSummary(marks);
    motion_controller_.shutdown();
    std::cout << "[CAL] done" << std::endl;
    return 0;
}

int AppController::runInteractiveServoConsole() {
    std::cout
        << "[SERVO] interactive servo console\n"
        << "[SERVO] commands: base <deg>, lower <deg>, upper <deg>, grip <deg>\n"
        << "[SERVO] extras: home, status, help\n"
        << "[SERVO] range: -90..+90 logical degrees\n"
        << "[SERVO] quit: Ctrl+C or type 'exit'\n";

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       kMotionChannelMap)) {
        std::cerr << "[SERVO] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[SERVO] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    SmokeJointOffsets current_offsets = kSmokeHomePose;
    bool initial_clamped = false;
    const MeArmJointTargets home_targets =
        makeSmokeTargets(current_offsets, initial_clamped);

    if (!motion_controller_.setTargets(home_targets)) {
        return fail(std::string("failed to stage home pose: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::cout << "[SERVO] home " << formatOffsets(current_offsets) << std::endl;

    const auto previous_sigint = std::signal(SIGINT, handleInteractiveServoSignal);
    const auto previous_sigterm = std::signal(SIGTERM, handleInteractiveServoSignal);
    g_interactive_servo_stop_requested = 0;

    auto restoreSignals = [&]() {
        std::signal(SIGINT, previous_sigint);
        std::signal(SIGTERM, previous_sigterm);
    };

    auto applyOffsets = [&](const SmokeJointOffsets& requested,
                            const std::string& label) -> bool {
        bool clamped = false;
        const SmokeJointOffsets bounded = clampSmokeOffsets(requested, clamped);
        const MeArmJointTargets targets = makeSmokeTargets(bounded, clamped);

        if (!motion_controller_.setTargets(targets)) {
            (void)fail(std::string("failed to set ") + label + ": " +
                       motion_controller_.lastErrorString());
            return false;
        }

        current_offsets = bounded;
        std::cout << "[SERVO] " << label << " " << formatOffsets(current_offsets);
        if (clamped) {
            std::cout << " [clamped]";
        }
        std::cout << std::endl;
        return true;
    };

    while (!g_interactive_servo_stop_requested) {
        std::cout << "servo> " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) {
            if (g_interactive_servo_stop_requested) {
                break;
            }
            if (std::cin.eof()) {
                break;
            }
            restoreSignals();
            return fail("stdin read failed");
        }

        std::istringstream iss(line);
        std::string command;
        if (!(iss >> command)) {
            continue;
        }

        command = toLowerCopy(command);
        if (command == "exit" || command == "quit") {
            break;
        }

        if (command == "help") {
            std::cout
                << "[SERVO] commands: base <deg>, lower <deg>, upper <deg>, grip <deg>, home, status, exit"
                << std::endl;
            continue;
        }

        if (command == "status") {
            std::cout << "[SERVO] " << formatOffsets(current_offsets)
                      << std::endl;
            continue;
        }

        if (command == "home") {
            if (!applyOffsets(kSmokeHomePose, "home")) {
                restoreSignals();
                return 1;
            }
            continue;
        }

        int angle = 0;
        if (!(iss >> angle)) {
            std::cout << "[SERVO] expected: <joint> <angle>" << std::endl;
            continue;
        }

        SmokeJointOffsets requested = current_offsets;
        if (command == "base") {
            requested.base = angle;
        } else if (command == "lower") {
            requested.lower = angle;
        } else if (command == "upper") {
            requested.upper = angle;
        } else if (command == "grip" || command == "gripper") {
            requested.grip = angle;
        } else {
            std::cout << "[SERVO] unknown joint: " << command << std::endl;
            continue;
        }

        std::ostringstream label;
        label << command << "=" << angle;
        if (!applyOffsets(requested, label.str())) {
            restoreSignals();
            return 1;
        }
    }

    restoreSignals();
    motion_controller_.shutdown();
    std::cout << "[SERVO] done" << std::endl;
    return 0;
}

int AppController::runMotionHomePose() {
    std::cout << "[HOME] setting all joints to 0" << std::endl;

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       kMotionChannelMap)) {
        std::cerr << "[HOME] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[HOME] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    bool clamped = false;
    const MeArmJointTargets home_targets =
        makeSmokeTargets(kSmokeHomePose, clamped);

    if (!motion_controller_.setTargets(home_targets)) {
        return fail(std::string("failed to stage home pose: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::cout << "[HOME] pose=HOME wait=2s";
    if (clamped) {
        std::cout << " [clamped]";
    }
    std::cout << std::endl;

    std::string timer_error;
    if (!waitForCppTimerDelay(kMotionHomeSettleDwell, timer_error)) {
        return fail(std::string("home dwell timer failed: ") + timer_error);
    }

    motion_controller_.shutdown();
    std::cout << "[HOME] done" << std::endl;
    return 0;
}

int AppController::runMotionSmokeTest(const MotionSmokeTestOptions& options) {
    std::cout
        << "[SMOKE] joint=" << smokeJointSelectionToString(options.joint)
        << " 0 -> -90 -> +90 -> 0"
        << "  wait=3s\n";

    MotionChannelMap channel_map{};
    channel_map.base = kBaseServoChannel;
    channel_map.lower = kLowerServoChannel;
    channel_map.upper = kUpperServoChannel;
    channel_map.gripper = kGripServoChannel;

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       channel_map)) {
        std::cerr << "[SMOKE] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[SMOKE] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    auto makeJointOffsets = [](std::size_t joint_index, int joint_offset) {
        SmokeJointOffsets offsets{};
        switch (joint_index) {
            case 0:
                offsets.base = joint_offset;
                break;
            case 1:
                offsets.lower = joint_offset;
                break;
            case 2:
                offsets.upper = joint_offset;
                break;
            case 3:
                offsets.grip = joint_offset;
                break;
            default:
                break;
        }
        return offsets;
    };

    auto stagePose = [&](const std::string& label,
                         const SmokeJointOffsets& requested) {
        bool clamped = false;
        const SmokeJointOffsets bounded = clampSmokeOffsets(requested, clamped);
        const MeArmJointTargets targets = makeSmokeTargets(bounded, clamped);

        if (!motion_controller_.setTargets(targets)) {
            return fail(std::string("failed to stage pose ") + label + ": " +
                        motion_controller_.lastErrorString());
        }

        std::cout << "[SMOKE] " << label;
        if (clamped) {
            std::cout << " [clamped]";
        }
        std::cout << std::endl;
        std::cout << "[SMOKE] wait 3s" << std::endl;

        std::string timer_error;
        if (!waitForCppTimerDelay(kSmokeStepDwell, timer_error)) {
            return fail(std::string("smoke dwell timer failed: ") + timer_error);
        }
        return 0;
    };

    bool home_clamped = false;
    const MeArmJointTargets home_targets =
        makeSmokeTargets(kSmokeHomePose, home_clamped);

    if (!motion_controller_.setTargets(home_targets)) {
        return fail(std::string("failed to stage home pose: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::cout << "[SMOKE] all -> 0" << std::endl;
    std::cout << "[SMOKE] wait 3s" << std::endl;
    {
        std::string timer_error;
        if (!waitForCppTimerDelay(kSmokeStepDwell, timer_error)) {
            return fail(std::string("initial smoke dwell timer failed: ") +
                        timer_error);
        }
    }

    const SmokeJointRunPlan plan = makeSmokeJointRunPlan(options.joint);
    if (plan.count == 0) {
        return fail("no smoke-test joint selected");
    }

    auto runJointSweep = [&](std::size_t index) {
        const SmokeJointSpec& spec = kSmokeJointSpecs[index];
        std::cout << "[SMOKE] " << spec.logical_name << std::endl;

        const std::string home_label = std::string(spec.logical_name) + " -> 0";
        const std::string neg_label = std::string(spec.logical_name) + " -> -90";
        const std::string pos_label = std::string(spec.logical_name) + " -> +90";

        if (stagePose(home_label, makeJointOffsets(index, 0)) != 0) {
            return 1;
        }
        if (stagePose(neg_label, makeJointOffsets(index, kSmokeNegativeStep)) != 0) {
            return 1;
        }
        if (stagePose(pos_label, makeJointOffsets(index, kSmokePositiveStep)) != 0) {
            return 1;
        }
        if (stagePose(home_label, makeJointOffsets(index, 0)) != 0) {
            return 1;
        }

        return 0;
    };

    for (std::size_t i = 0; i < plan.count; ++i) {
        if (runJointSweep(plan.indices[i]) != 0) {
            motion_controller_.shutdown();
            return 1;
        }
    }

    motion_controller_.shutdown();
    std::cout << "[SMOKE] done" << std::endl;
    return 0;
}

int AppController::runCameraBackendCheck(const LiveTestOptions& options) {
    constexpr int kValidationFrameTarget = 60;
    constexpr int kMaxConsecutiveFailures = 10;

    std::cout << "[CAMERA_CHECK] camera=" << options.camera_index
              << " backend_request="
              << cameraBackendPreferenceToString(options.backend_preference)
              << " frames=" << kValidationFrameTarget << std::endl;

    CameraCapture camera_capture(
        CameraCapture::Options{options.camera_index, options.backend_preference});
    std::cout << "[CAMERA_CHECK] backend_active="
              << camera_capture.backendImplementation() << " ("
              << camera_capture.backendName() << ")" << std::endl;

    FrameEvent frame_event;
    int frames_received = 0;
    int frame_failures = 0;
    int consecutive_failures = 0;
    int frame_width = 0;
    int frame_height = 0;
    std::optional<std::chrono::steady_clock::time_point> first_capture_timestamp;
    std::optional<std::chrono::steady_clock::time_point> last_capture_timestamp;

    const auto validation_start = std::chrono::steady_clock::now();
    while (frames_received < kValidationFrameTarget &&
           consecutive_failures < kMaxConsecutiveFailures) {
        if (!camera_capture.waitForNextFrame(frame_event)) {
            ++frame_failures;
            ++consecutive_failures;
            std::cerr << "[CAMERA_CHECK] frame failure " << frame_failures
                      << " (consecutive=" << consecutive_failures << ")"
                      << std::endl;
            continue;
        }

        ++frames_received;
        consecutive_failures = 0;
        frame_width = frameWidth(frame_event.image_data);
        frame_height = frameHeight(frame_event.image_data);
        if (!first_capture_timestamp.has_value()) {
            first_capture_timestamp = frame_event.capture_timestamp;
            std::cout << "[CAMERA_CHECK] first_frame_ms="
                      << elapsedMilliseconds(validation_start,
                                             *first_capture_timestamp)
                      << " size=" << frame_width << "x" << frame_height
                      << std::endl;
        }
        last_capture_timestamp = frame_event.capture_timestamp;
    }

    if (frames_received == 0) {
        std::cerr << "[CAMERA_CHECK] no frames received" << std::endl;
        return 1;
    }

    const long long sample_window_ms =
        (first_capture_timestamp.has_value() && last_capture_timestamp.has_value())
            ? elapsedMilliseconds(*first_capture_timestamp,
                                  *last_capture_timestamp)
            : 0;
    const double approx_fps =
        (frames_received > 1 && sample_window_ms > 0)
            ? (1000.0 * static_cast<double>(frames_received - 1) /
               static_cast<double>(sample_window_ms))
            : 0.0;

    std::ostringstream fps_stream;
    fps_stream << std::fixed << std::setprecision(1) << approx_fps;

    std::cout << "[CAMERA_CHECK] summary: frames=" << frames_received
              << " failures=" << frame_failures
              << " sample_window_ms=" << sample_window_ms
              << " approx_fps=" << fps_stream.str()
              << " backend=" << camera_capture.backendImplementation() << " ("
              << camera_capture.backendName() << ")" << std::endl;

    if (consecutive_failures >= kMaxConsecutiveFailures) {
        std::cerr << "[CAMERA_CHECK] failed after repeated capture errors"
                  << std::endl;
        return 1;
    }

    return 0;
}

int AppController::runLiveMarkerTest(const LiveTestOptions& options) {
    std::cout
        << "[LIVE_TEST] Starting live marker safety test mode\n"
        << "[LIVE_TEST] Camera index: " << options.camera_index << "\n"
        << "[LIVE_TEST] Expected marker ID: " << options.expected_marker_id << "\n"
        << "[LIVE_TEST] Camera backend preference: "
        << cameraBackendPreferenceToString(options.backend_preference) << "\n"
        << "[LIVE_TEST] Auto operator ack: "
        << (options.auto_ack ? "ON" : "OFF") << "\n"
        << "[LIVE_TEST] Physical button = single-button control\n"
        << "[LIVE_TEST] Controls: space/button=control, 0/1/2/3=mode/routine, esc=quit\n"
        << "[LIVE_TEST] Manual mode keys: d/a=base left/right, w/s=forward/back, i/k=up/down, l/j=open/close\n"
        << "[LIVE_TEST] Starting in DISARMED setup mode\n"
        << "[LIVE_TEST] Guardian thresholds: freeze after "
        << kLiveFreezeBadFrameThreshold
        << " consecutive bad frames, recover after "
        << kLiveRecoverGoodFrameThreshold
        << " consecutive good frames.\n"
        << "[LIVE_TEST] Modes: 0=MANUAL, 1=SURGERY_CUT, 2=BASE_SCAN, 3=GRIP_PULSE\n"
        << "[LIVE_TEST] Focus debug enabled: FOCUS_SCORE (Laplacian variance), "
           "higher usually means sharper marker edges.\n";

    MotionChannelMap channel_map{};
    channel_map.base = kBaseServoChannel;
    channel_map.lower = kLowerServoChannel;
    channel_map.upper = kUpperServoChannel;
    channel_map.gripper = kGripServoChannel;

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       channel_map)) {
        std::cerr << "[LIVE_TEST] Motion controller initialization failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    MotionControllerHardwareAdapter hardware(motion_controller_);
    VisionConfig vision_config;
    vision_config.expectedMarkerId = options.expected_marker_id;
    VisionProcessor vision_processor(vision_config);
    CameraCapture camera_capture(
        CameraCapture::Options{options.camera_index, options.backend_preference});
    std::cout << "[LIVE_TEST] Camera backend active: "
              << camera_capture.backendImplementation() << " ("
              << camera_capture.backendName() << ")" << std::endl;

    std::unique_ptr<GuardianStateMachine> guardian;
    std::unique_ptr<RobotInterlock> interlock;
    bool guardian_armed = false;
    bool motion_faulted = false;
    bool motion_gate_open = false;
    FreezeReason pending_reason = FreezeReason::UNKNOWN_FAULT;
    FreezeReason pending_freeze_reason = FreezeReason::UNKNOWN_FAULT;
    SafetyState current_vision_state = SafetyState::SAFE;
    bool frame_is_safe = false;
    bool waiting_for_ack = false;
    bool waiting_for_ack_announced = false;
    bool home_pose_staged = false;
    std::string current_pose_name = "HOME";
    std::size_t selected_routine_index = 1;
    std::size_t next_routine_step_index = 0;
    SmokeJointOffsets current_pose_offsets = kSmokeHomePose;
    bool current_pose_offsets_known = false;
    SmokeJointOffsets pose_slew_target = kSmokeHomePose;
    bool pose_slew_active = false;
    std::chrono::steady_clock::time_point next_pose_slew_due =
        std::chrono::steady_clock::now();
    bool freeze_command_pending = false;
    std::optional<std::chrono::steady_clock::time_point> freeze_command_due;
    ControllerEventQueue control_events;
    CppTimerCallback live_step_timer;
    bool live_step_timer_started = false;
    RuntimeLatencyMetrics latency_metrics;
    constexpr std::size_t kVisionHistoryLimit = 180;
    constexpr std::size_t kEventHistoryLimit = 64;
    std::deque<double> vision_us_history;
    std::deque<double> unsafe_detect_history_ms;
    std::deque<double> freeze_pipeline_history_ms;
    std::deque<double> freeze_cmd_history_ms;
    std::deque<double> total_stop_history_ms;
    std::deque<double> ack_resume_history_ms;
    auto appendHistorySample = [&](std::deque<double>& history,
                                   double value,
                                   std::size_t limit) {
        history.push_back(value);
        if (history.size() > limit) {
            history.pop_front();
        }
    };
    auto copyHistory = [&](const std::deque<double>& history) {
        return std::vector<double>(history.begin(), history.end());
    };
    std::optional<std::chrono::steady_clock::time_point>
        pending_unsafe_capture_timestamp;
    std::optional<std::chrono::steady_clock::time_point>
        pending_unsafe_decision_timestamp;
    std::optional<std::chrono::steady_clock::time_point> ack_request_timestamp;

    auto applyPoseOffsets = [&](const SmokeJointOffsets& offsets,
                                const char* context) -> bool {
        bool target_clamped = false;
        const MeArmJointTargets targets = makeDemoTargets(offsets, target_clamped);
        if (!motion_controller_.setTargets(targets)) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] failed to set pose (" << context << "): "
                      << motion_controller_.lastErrorString() << std::endl;
            return false;
        }
        return true;
    };

    auto stagePose = [&](const DemoPoseStep& step) -> bool {
        bool clamped = false;
        const SmokeJointOffsets target_offsets =
            clampDemoOffsets(step.offsets, clamped);

        if (!current_pose_offsets_known) {
            current_pose_offsets = target_offsets;
            current_pose_offsets_known = true;
            if (!applyPoseOffsets(current_pose_offsets, "initial")) {
                return false;
            }
        }

        pose_slew_target = target_offsets;
        pose_slew_active =
            (current_pose_offsets.base != pose_slew_target.base) ||
            (current_pose_offsets.lower != pose_slew_target.lower) ||
            (current_pose_offsets.upper != pose_slew_target.upper) ||
            (current_pose_offsets.grip != pose_slew_target.grip);
        next_pose_slew_due = std::chrono::steady_clock::now();

        if (!pose_slew_active) {
            if (!applyPoseOffsets(pose_slew_target, "hold")) {
                return false;
            }
        }

        current_pose_name = step.name;
        std::cout << "[LIVE_TEST] pose=" << step.name;
        if (clamped) {
            std::cout << " [clamped]";
        }
        if (pose_slew_active) {
            std::cout << " [slew]";
        }
        std::cout << " wait=1s" << std::endl;
        return true;
    };

    auto advancePoseSlew = [&]() -> bool {
        if (!pose_slew_active) {
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < next_pose_slew_due) {
            return true;
        }

        auto stepTowards = [&](int current, int target) {
            if (current < target) {
                return std::min(current + kDemoSlewStepDegrees, target);
            }
            if (current > target) {
                return std::max(current - kDemoSlewStepDegrees, target);
            }
            return current;
        };

        SmokeJointOffsets next_offsets = current_pose_offsets;
        next_offsets.base = stepTowards(next_offsets.base, pose_slew_target.base);
        next_offsets.lower = stepTowards(next_offsets.lower, pose_slew_target.lower);
        next_offsets.upper = stepTowards(next_offsets.upper, pose_slew_target.upper);
        next_offsets.grip = stepTowards(next_offsets.grip, pose_slew_target.grip);

        if (!applyPoseOffsets(next_offsets, "slew")) {
            return false;
        }

        current_pose_offsets = next_offsets;
        pose_slew_active =
            (current_pose_offsets.base != pose_slew_target.base) ||
            (current_pose_offsets.lower != pose_slew_target.lower) ||
            (current_pose_offsets.upper != pose_slew_target.upper) ||
            (current_pose_offsets.grip != pose_slew_target.grip);
        next_pose_slew_due = now + kDemoSlewStepInterval;
        return true;
    };

    auto stopLiveStepTimer = [&]() {
        if (live_step_timer_started) {
            live_step_timer.stop();
            live_step_timer_started = false;
        }
    };

    auto startLiveStepTimer = [&]() -> bool {
        if (live_step_timer_started) {
            return true;
        }

        live_step_timer.registerEventCallback(
            [&]() { control_events.pushLiveStepReady(); });
        try {
            live_step_timer.startms(static_cast<long>(kDemoStepDwell.count()), PERIODIC);
        } catch (const char* exception) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] routine timer failed: " << exception
                      << std::endl;
            return false;
        } catch (...) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] routine timer failed" << std::endl;
            return false;
        }

        live_step_timer_started = true;
        return true;
    };

    auto announceWaitingState = [&]() {
        const bool should_wait =
            frame_is_safe && guardian_armed &&
            guardian->getState() == GuardianState::FROZEN_UNSAFE;

        if (should_wait && !waiting_for_ack_announced) {
            std::cout << "[LIVE_TEST] waiting for continue" << std::endl;
            waiting_for_ack_announced = true;
        }

        waiting_for_ack = should_wait;
        if (!should_wait) {
            waiting_for_ack_announced = false;
        }
    };

    auto selectRoutine = [&](std::size_t routine_index) -> bool {
        const LiveRoutineDefinition routine = getLiveRoutineDefinition(routine_index);
        stopLiveStepTimer();
        selected_routine_index = routine_index;
        next_routine_step_index = 0;
        std::cout << "[LIVE_TEST] routine " << routine.number << " selected: "
                  << routine.name << std::endl;

        if (guardian_armed && motion_gate_open && frame_is_safe) {
            if (routine.auto_progress) {
                if (!stagePose(kDemoHomeStep)) {
                    return false;
                }
                home_pose_staged = true;
                if (!startLiveStepTimer()) {
                    return false;
                }
            } else {
                std::cout << "[LIVE_TEST] manual mode active: auto routine paused."
                          << std::endl;
            }
        }

        return true;
    };

    auto resetEnforcementState = [&]() {
        pending_reason = FreezeReason::UNKNOWN_FAULT;
        pending_freeze_reason = FreezeReason::UNKNOWN_FAULT;
        freeze_command_pending = false;
        freeze_command_due.reset();
        guardian = std::make_unique<GuardianStateMachine>(
            kLiveFreezeBadFrameThreshold,
            kLiveRecoverGoodFrameThreshold);
        interlock = std::make_unique<RobotInterlock>(hardware);

        guardian->setOnFreezeCallback([&]() {
            const auto freeze_callback_start = std::chrono::steady_clock::now();
            if (pending_unsafe_decision_timestamp.has_value()) {
                latency_metrics.freeze_pipeline_ms = elapsedMilliseconds(
                    *pending_unsafe_decision_timestamp,
                    freeze_callback_start);
                appendHistorySample(freeze_pipeline_history_ms,
                                    static_cast<double>(
                                        *latency_metrics.freeze_pipeline_ms),
                                    kEventHistoryLimit);
            }
            stopLiveStepTimer();
            next_routine_step_index = 0;
            pending_freeze_reason = pending_reason;
            if (guardian_armed) {
                std::cout << "[LIVE_TEST] unsafe received: retracting to safe pose"
                          << std::endl;
                if (!stagePose(kSurgeryRetractStep)) {
                    motion_faulted = true;
                }
            }
            freeze_command_pending = true;
            freeze_command_due =
                std::chrono::steady_clock::now() + kSurgeryRetractDwell;
            motion_gate_open = false;
            waiting_for_ack = false;
            waiting_for_ack_announced = false;
            std::cout
                << "[LIVE_TEST] freeze pending: will hold after retract dwell"
                << std::endl;
        });

        guardian->setOnClearFreezeCallback([&]() {
            interlock->onControlEvent(ControlEvent::ALLOW_MOTION);
            const auto resume_callback_end = std::chrono::steady_clock::now();
            if (ack_request_timestamp.has_value()) {
                latency_metrics.ack_to_resume_ms = elapsedMilliseconds(
                    *ack_request_timestamp,
                    resume_callback_end);
                appendHistorySample(ack_resume_history_ms,
                                    static_cast<double>(
                                        *latency_metrics.ack_to_resume_ms),
                                    kEventHistoryLimit);
            }
            ack_request_timestamp.reset();
            if (interlock->state() == InterlockState::FAULT) {
                motion_faulted = true;
                return;
            }
            if (getLiveRoutineDefinition(selected_routine_index).auto_progress) {
                if (!startLiveStepTimer()) {
                    motion_faulted = true;
                    return;
                }
            }
            motion_gate_open = true;
            std::cout << "[LIVE_TEST] resume" << std::endl;
            logLiveLatencySample("resume", latency_metrics);
        });

        guardian->setOnStateChangeCallback([&](GuardianState from, GuardianState to) {
            std::cout << "[GUARDIAN] " << guardian->stateToString(from) << " -> "
                      << guardian->stateToString(to) << std::endl;
        });
    };

    resetEnforcementState();

    auto processPendingFreezeCommand = [&]() -> bool {
        if (!freeze_command_pending || !freeze_command_due.has_value()) {
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < *freeze_command_due) {
            return true;
        }

        freeze_command_pending = false;
        freeze_command_due.reset();
        const auto freeze_cmd_start = std::chrono::steady_clock::now();
        interlock->onControlEvent(ControlEvent::FREEZE_NOW, pending_freeze_reason);
        const auto freeze_cmd_end = std::chrono::steady_clock::now();
        latency_metrics.freeze_cmd_ms = elapsedMilliseconds(freeze_cmd_start,
                                                            freeze_cmd_end);
        appendHistorySample(freeze_cmd_history_ms,
                            static_cast<double>(*latency_metrics.freeze_cmd_ms),
                            kEventHistoryLimit);
        if (pending_unsafe_capture_timestamp.has_value()) {
            latency_metrics.total_stop_ms = elapsedMilliseconds(
                *pending_unsafe_capture_timestamp,
                freeze_cmd_end);
            appendHistorySample(total_stop_history_ms,
                                static_cast<double>(*latency_metrics.total_stop_ms),
                                kEventHistoryLimit);
        }
        pending_unsafe_capture_timestamp.reset();
        pending_unsafe_decision_timestamp.reset();
        std::cout << "[LIVE_TEST] freeze: "
                  << freezeReasonToString(pending_freeze_reason) << std::endl;
        logLiveLatencySample("freeze", latency_metrics);

        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            return false;
        }
        return true;
    };

    PhysicalButtonModule button_module;
    std::cout << "[LIVE_TEST] Physical button module: "
              << (button_module.available() ? "configured" : "disabled");
    const char* button_module_status =
        button_module.available() ? button_module.statusString()
                                  : button_module.lastErrorString();
    if (button_module_status != nullptr &&
        std::string(button_module_status) != "no error" &&
        std::string(button_module_status) != "no status") {
        std::cout << " (" << button_module_status << ")";
    }
    std::cout << std::endl;
    if (!button_module.available()) {
        std::cout << "[LIVE_TEST] Configure ARGUS_BUTTON_ACK_GPIO to enable the "
                     "physical single-button control."
                  << std::endl;
    }

    auto freezeMotionBeforeModeChange = [&](const char* action_label) -> bool {
        interlock->onControlEvent(ControlEvent::FREEZE_NOW, FreezeReason::UNKNOWN_FAULT);
        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Unable to freeze motion before " << action_label
                      << ": " << motion_controller_.lastErrorString() << std::endl;
            return false;
        }

        return true;
    };

    auto requestArm = [&]() -> bool {
        if (guardian_armed) {
            std::cout << "[LIVE_TEST] ARM request ignored: guardian already armed."
                      << std::endl;
            return true;
        }

        if (!frame_is_safe) {
            std::cout << "[LIVE_TEST] ARM request rejected: current observed condition is "
                         "unsafe (vision="
                      << safetyStateToString(current_vision_state) << ")." << std::endl;
            return true;
        }

        if (!freezeMotionBeforeModeChange("arming")) {
            return false;
        }

        resetEnforcementState();
        if (!stagePose(kDemoHomeStep)) {
            return false;
        }
        home_pose_staged = true;
        next_routine_step_index = 0;

        interlock->onControlEvent(ControlEvent::FREEZE_NOW, FreezeReason::UNKNOWN_FAULT);
        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Unable to prepare motion path for arm: "
                      << motion_controller_.lastErrorString() << std::endl;
            return false;
        }

        interlock->operatorAcknowledge();
        interlock->onControlEvent(ControlEvent::ALLOW_MOTION);
        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Unable to enable motion on arm: "
                      << motion_controller_.lastErrorString() << std::endl;
            return false;
        }

        stopLiveStepTimer();
        if (getLiveRoutineDefinition(selected_routine_index).auto_progress) {
            if (!startLiveStepTimer()) {
                return false;
            }
        }
        guardian_armed = true;
        motion_gate_open = true;
        waiting_for_ack = false;
        waiting_for_ack_announced = false;
        ack_request_timestamp.reset();
        pending_unsafe_capture_timestamp.reset();
        pending_unsafe_decision_timestamp.reset();
        freeze_command_pending = false;
        freeze_command_due.reset();
        std::cout << "[LIVE_TEST] ARM accepted: guardian enforcement is now ACTIVE."
                  << std::endl;
        return true;
    };

    auto requestDisarm = [&]() -> bool {
        if (!guardian_armed) {
            std::cout << "[LIVE_TEST] DISARM request ignored: guardian already disarmed."
                      << std::endl;
            return true;
        }

        if (!freezeMotionBeforeModeChange("disarming")) {
            return false;
        }

        guardian_armed = false;
        motion_gate_open = false;
        waiting_for_ack = false;
        waiting_for_ack_announced = false;
        home_pose_staged = false;
        ack_request_timestamp.reset();
        pending_unsafe_capture_timestamp.reset();
        pending_unsafe_decision_timestamp.reset();
        freeze_command_pending = false;
        freeze_command_due.reset();
        pose_slew_active = false;
        stopLiveStepTimer();
        next_routine_step_index = 0;
        current_pose_name = "HOLD";
        resetEnforcementState();
        std::cout << "[LIVE_TEST] DISARM accepted: guardian enforcement is now INACTIVE "
                     "(setup/observation mode)."
                  << std::endl;
        return true;
    };

    auto requestAcknowledge = [&]() -> bool {
        if (!guardian_armed) {
            std::cout << "[LIVE_TEST] ACK request ignored: guardian is disarmed."
                      << std::endl;
            return true;
        }

        if (freeze_command_pending) {
            std::cout << "[LIVE_TEST] ACK request ignored: retract in progress."
                      << std::endl;
            return true;
        }

        if (!frame_is_safe) {
            std::cout << "[LIVE_TEST] ACK request ignored: current observed condition is "
                         "unsafe (vision="
                      << safetyStateToString(current_vision_state) << ")." << std::endl;
            return true;
        }

        if (guardian->getState() != GuardianState::FROZEN_UNSAFE) {
            std::cout << "[LIVE_TEST] ACK request ignored: guardian is not frozen."
                      << std::endl;
            return true;
        }

        guardian->operatorAcknowledge();
        interlock->operatorAcknowledge();
        ack_request_timestamp = std::chrono::steady_clock::now();
        std::cout << "[LIVE_TEST] Operator acknowledge requested." << std::endl;
        return true;
    };

    auto requestContinue = [&]() -> bool {
        if (!guardian_armed) {
            return requestArm();
        }
        if (guardian->getState() == GuardianState::FROZEN_UNSAFE) {
            return requestAcknowledge();
        }
        if (guardian->getState() == GuardianState::RESET_PENDING) {
            std::cout << "[LIVE_TEST] CONTINUE ignored: waiting for recovery"
                      << std::endl;
            return true;
        }
        return requestDisarm();
    };

    auto requestFromButton = [&](PhysicalButtonEvent event) -> bool {
        std::cout << "[BUTTON] " << PhysicalButtonModule::eventToString(event)
                  << std::endl;
        switch (event) {
            case PhysicalButtonEvent::ARM_REQUEST:
            case PhysicalButtonEvent::ACK_REQUEST:
                return requestContinue();
            case PhysicalButtonEvent::DISARM_REQUEST:
                return requestDisarm();
        }
        return true;
    };

    auto requestManualNudge = [&](int delta_base,
                                  int delta_lower,
                                  int delta_upper,
                                  int delta_grip,
                                  const char* label) -> bool {
        const LiveRoutineDefinition routine =
            getLiveRoutineDefinition(selected_routine_index);
        if (routine.auto_progress) {
            return true;
        }

        if (!guardian_armed || !motion_gate_open || waiting_for_ack ||
            freeze_command_pending ||
            guardian->getState() != GuardianState::SAFE_MONITORING) {
            std::cout << "[LIVE_TEST] manual nudge ignored: control not available"
                      << std::endl;
            return true;
        }

        if (!current_pose_offsets_known) {
            current_pose_offsets = kSmokeHomePose;
            current_pose_offsets_known = true;
            pose_slew_target = current_pose_offsets;
        }

        SmokeJointOffsets target_offsets =
            pose_slew_active ? pose_slew_target : current_pose_offsets;
        target_offsets.base += delta_base;
        target_offsets.lower += delta_lower;
        target_offsets.upper += delta_upper;
        target_offsets.grip += delta_grip;

        bool clamped = false;
        target_offsets = clampDemoOffsets(target_offsets, clamped);
        pose_slew_target = target_offsets;
        pose_slew_active =
            (current_pose_offsets.base != pose_slew_target.base) ||
            (current_pose_offsets.lower != pose_slew_target.lower) ||
            (current_pose_offsets.upper != pose_slew_target.upper) ||
            (current_pose_offsets.grip != pose_slew_target.grip);
        next_pose_slew_due = std::chrono::steady_clock::now();
        current_pose_name = "MANUAL";

        if (pose_slew_active) {
            std::cout << "[LIVE_TEST] manual " << label;
            if (clamped) {
                std::cout << " [clamped]";
            }
            std::cout << " [slew]" << std::endl;
        }
        return true;
    };

    FrameEvent latest_frame_event;
    bool latest_frame_available = false;
    std::uint64_t frame_index = 0;
    bool processed_any_frame = false;
    int consecutive_capture_failures = 0;
    double focus_score = 0.0;
    const char* focus_quality = "BLURRY";

    auto handleControlEvent =
        [&](const ControllerEvent& event) -> ControllerEventDisposition {
        switch (event.kind) {
            case ControllerEventKind::ButtonInput:
                return requestFromButton(*event.button_event)
                           ? ControllerEventDisposition::Consumed
                           : ControllerEventDisposition::Abort;
            case ControllerEventKind::LiveStepReady: {
                if (!guardian_armed || !motion_gate_open || waiting_for_ack ||
                    pose_slew_active || freeze_command_pending) {
                    return ControllerEventDisposition::Deferred;
                }

                const LiveRoutineDefinition routine =
                    getLiveRoutineDefinition(selected_routine_index);
                if (!routine.auto_progress || routine.steps == nullptr ||
                    routine.step_count == 0) {
                    return ControllerEventDisposition::Consumed;
                }
                const DemoPoseStep& step = routine.steps[next_routine_step_index];
                if (!stagePose(step)) {
                    return ControllerEventDisposition::Abort;
                }

                next_routine_step_index =
                    (next_routine_step_index + 1) % routine.step_count;
                return ControllerEventDisposition::Consumed;
            }
            case ControllerEventKind::DemoStepReady:
                return ControllerEventDisposition::Consumed;
            case ControllerEventKind::FrameCaptureFailed:
                ++consecutive_capture_failures;
                std::cerr << "[LIVE_TEST] Frame capture failed ("
                          << consecutive_capture_failures
                          << "/30). Retrying..." << std::endl;
                if (consecutive_capture_failures >= 30) {
                    return ControllerEventDisposition::Abort;
                }
                return ControllerEventDisposition::Consumed;
            case ControllerEventKind::FrameAvailable: {
                consecutive_capture_failures = 0;
                processed_any_frame = true;
                latest_frame_event = *event.frame_event;
                latest_frame_available = !latest_frame_event.image_data.empty();
                ++frame_index;

                const SafetyResult result = vision_processor.process(
                    latest_frame_event.image_data,
                    latest_frame_event.capture_timestamp);
                latency_metrics.vision_us = result.processing_time.count();
                appendHistorySample(vision_us_history,
                                    static_cast<double>(latency_metrics.vision_us),
                                    kVisionHistoryLimit);
                focus_score = computeFocusScore(latest_frame_event.image_data);
                focus_quality = focusQualityLabel(focus_score);

                current_vision_state = result.state;
                frame_is_safe = (current_vision_state == SafetyState::SAFE);
                if (!frame_is_safe) {
                    pending_reason = mapSafetyToFreezeReason(current_vision_state);
                }

                const GuardianState guardian_state_before_update =
                    guardian->getState();
                if (guardian_armed &&
                    guardian_state_before_update ==
                        GuardianState::SAFE_MONITORING) {
                    if (!frame_is_safe) {
                        if (!pending_unsafe_capture_timestamp.has_value()) {
                            latency_metrics.unsafe_detect_ms.reset();
                            latency_metrics.freeze_pipeline_ms.reset();
                            latency_metrics.freeze_cmd_ms.reset();
                            latency_metrics.total_stop_ms.reset();
                            latency_metrics.ack_to_resume_ms.reset();
                            pending_unsafe_capture_timestamp =
                                latest_frame_event.capture_timestamp;
                            pending_unsafe_decision_timestamp = result.timestamp;
                            latency_metrics.unsafe_detect_ms = elapsedMilliseconds(
                                latest_frame_event.capture_timestamp,
                                result.timestamp);
                            appendHistorySample(unsafe_detect_history_ms,
                                                static_cast<double>(
                                                    *latency_metrics
                                                         .unsafe_detect_ms),
                                                kEventHistoryLimit);
                            logLiveLatencySample("unsafe_detect",
                                                 latency_metrics);
                        }
                    } else {
                        pending_unsafe_capture_timestamp.reset();
                        pending_unsafe_decision_timestamp.reset();
                    }
                }
                if (guardian_armed) {
                    guardian->processFrame(frame_is_safe ? FrameStatus::FRAME_GOOD
                                                         : FrameStatus::FRAME_BAD);
                }
                announceWaitingState();

                if (guardian_armed && options.auto_ack &&
                    !freeze_command_pending &&
                    guardian->getState() == GuardianState::FROZEN_UNSAFE) {
                    ack_request_timestamp = std::chrono::steady_clock::now();
                    guardian->operatorAcknowledge();
                    interlock->operatorAcknowledge();
                    std::cout
                        << "[LIVE_TEST] Operator acknowledge sent automatically"
                        << std::endl;
                }

                if (guardian_armed) {
                    interlock->guardianHeartbeat(
                        static_cast<std::uint32_t>(frame_index));
                }

                return ControllerEventDisposition::Consumed;
            }
        }
        return ControllerEventDisposition::Consumed;
    };

    cv::namedWindow(kLiveCameraWindowName, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(kLiveStatusWindowName, cv::WINDOW_AUTOSIZE);

    struct LiveStatusSnapshot {
        bool guardian_armed = false;
        bool scene_safe = false;
        bool waiting_for_continue = false;
        int routine_number = 0;
        std::string vision_state;
        std::string guardian_state;
        std::string interlock_state;
        std::string motion_controller_state;
        std::string freeze_reason;
    };

    LiveStatusSnapshot last_live_status;
    bool live_status_known = false;
    auto maybeLogLiveStatus = [&](const LiveStatusSnapshot& snapshot) {
        const bool changed =
            !live_status_known ||
            snapshot.guardian_armed != last_live_status.guardian_armed ||
            snapshot.scene_safe != last_live_status.scene_safe ||
            snapshot.waiting_for_continue !=
                last_live_status.waiting_for_continue ||
            snapshot.routine_number != last_live_status.routine_number ||
            snapshot.vision_state != last_live_status.vision_state ||
            snapshot.guardian_state != last_live_status.guardian_state ||
            snapshot.interlock_state != last_live_status.interlock_state ||
            snapshot.motion_controller_state !=
                last_live_status.motion_controller_state ||
            snapshot.freeze_reason != last_live_status.freeze_reason;
        if (!changed) {
            return;
        }

        std::cout << "[LIVE_TEST] status: armed="
                  << (snapshot.guardian_armed ? "YES" : "NO")
                  << " safe=" << (snapshot.scene_safe ? "YES" : "NO")
                  << " waiting="
                  << (snapshot.waiting_for_continue ? "YES" : "NO")
                  << " routine=" << snapshot.routine_number
                  << " vision=" << snapshot.vision_state
                  << " guardian=" << snapshot.guardian_state
                  << " interlock=" << snapshot.interlock_state
                  << " motion_ctrl=" << snapshot.motion_controller_state
                  << " freeze_reason=" << snapshot.freeze_reason << std::endl;

        last_live_status = snapshot;
        live_status_known = true;
    };

    std::atomic<bool> capture_stop_requested{false};
    std::thread capture_thread([&]() {
        while (!capture_stop_requested.load(std::memory_order_relaxed)) {
            FrameEvent captured_frame;
            if (camera_capture.waitForNextFrame(captured_frame)) {
                control_events.pushFrame(std::move(captured_frame));
                continue;
            }

            control_events.pushFrameCaptureFailed();

            std::string timer_error;
            if (!waitForCppTimerDelay(kCaptureRetryBackoff, timer_error)) {
                control_events.pushFrameCaptureFailed();
                break;
            }
        }
    });

    constexpr int kLiveStatusWidth = 430;
    constexpr int kLiveMetricsWidth = 430;
    cv::Mat status_frame;
    cv::Mat metrics_frame;

    while (true) {
        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Interlock entered FAULT state (motion controller: "
                      << motion_controller_.lastErrorString() << ")" << std::endl;
            break;
        }
        (void)control_events.waitForEvents(std::chrono::milliseconds(5));

        std::string guardian_state_text = "DISARMED_SETUP";
        std::string interlock_state_text = "DISARMED";
        std::string freeze_reason_text = "N/A";

        PhysicalButtonEvent button_event;
        while (button_module.poll(button_event)) {
            control_events.pushButton(button_event);
        }

        if (!control_events.drain(handleControlEvent)) {
            motion_faulted = true;
            break;
        }

        if (!advancePoseSlew()) {
            motion_faulted = true;
            break;
        }

        if (!processPendingFreezeCommand()) {
            motion_faulted = true;
            break;
        }

        if (guardian_armed) {
            guardian_state_text = guardian->getCurrentStateString();
            interlock_state_text = interlockStateToString(interlock->state());
            freeze_reason_text = freezeReasonToString(interlock->freezeReason());
        }

        maybeLogLiveStatus(LiveStatusSnapshot{
            guardian_armed,
            frame_is_safe,
            waiting_for_ack,
            getLiveRoutineDefinition(selected_routine_index).number,
            safetyStateToString(current_vision_state),
            guardian_state_text,
            interlock_state_text,
            motionControllerStateToString(motion_controller_.outputState()),
            freeze_reason_text});

        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Motion control fault detected: "
                      << motion_controller_.lastErrorString() << std::endl;
            break;
        }

        cv::Mat display_frame;
        if (latest_frame_available) {
            display_frame = latest_frame_event.image_data.clone();
        } else {
            display_frame = makePlaceholderFrame(640, 480);
        }
        const bool decision_is_safe = guardian_armed
                                          ? ((current_vision_state == SafetyState::SAFE) &&
                                             (guardian->getState() ==
                                              GuardianState::SAFE_MONITORING) &&
                                             interlock->motionAllowed())
                                          : (current_vision_state == SafetyState::SAFE);
        std::string ui_state_label = "SETUP";
        std::string ui_state_description = "Waiting for safe scene";
        std::string next_action = "MAKE SCENE SAFE";
        if (!guardian_armed && frame_is_safe) {
            ui_state_label = "READY";
            ui_state_description = "Scene safe and ready to arm";
            next_action = "PRESS CONTROL TO ARM";
        } else if (guardian_armed && motion_gate_open && decision_is_safe) {
            ui_state_label = "RUNNING";
            ui_state_description = "Guard active and motion allowed";
            next_action = "PRESS CONTROL TO DISARM";
        } else if (guardian_armed &&
                   guardian->getState() == GuardianState::FROZEN_UNSAFE) {
            ui_state_label = "FROZEN";
            ui_state_description = "Unsafe detected, motion stopped";
            next_action = frame_is_safe ? "PRESS CONTROL TO RESUME"
                                        : "CLEAR WORKSPACE";
        } else if (guardian_armed &&
                   guardian->getState() == GuardianState::RESET_PENDING) {
            ui_state_label = "WAITING";
            ui_state_description = "Safe again, waiting for recovery";
            next_action = "WAIT FOR RECOVERY";
        } else if (guardian_armed && frame_is_safe) {
            ui_state_label = "READY";
            ui_state_description = "Guard armed, motion blocked";
            next_action = "PRESS CONTROL";
        }

        const cv::Scalar focus_color =
            (focus_score < 60.0) ? cv::Scalar(60, 60, 200)
                                 : ((focus_score < 180.0) ? cv::Scalar(50, 180, 230)
                                                           : cv::Scalar(60, 170, 80));
        const std::string routine_label =
            std::to_string(
                getLiveRoutineDefinition(selected_routine_index).number) +
            " " + getLiveRoutineDefinition(selected_routine_index).name;
        const std::string footer_info =
            std::to_string(frameWidth(display_frame)) + "x" +
            std::to_string(frameHeight(display_frame)) + " | marker " +
            std::to_string(options.expected_marker_id) + " | " +
            camera_capture.backendName();

        SupervisoryUiModel live_ui{};
        live_ui.mode_title = "LIVE TEST";
        live_ui.state_label = ui_state_label;
        live_ui.state_description = ui_state_description;
        live_ui.state_color =
            guardian_armed && guardian->getState() == GuardianState::FROZEN_UNSAFE
                ? cv::Scalar(60, 60, 200)
                : (frame_is_safe ? cv::Scalar(60, 170, 80)
                                 : cv::Scalar(170, 140, 60));
        live_ui.motion_label = motion_gate_open ? "ALLOWED" : "BLOCKED";
        live_ui.motion_color =
            motion_gate_open ? cv::Scalar(60, 170, 80)
                             : (frame_is_safe ? cv::Scalar(50, 180, 230)
                                              : cv::Scalar(60, 60, 200));
        live_ui.operator_prompt = "space/button = control";
        live_ui.next_action = next_action;
        live_ui.freeze_reason = freeze_reason_text;
        live_ui.footer_info = footer_info;
        live_ui.latency = latency_metrics;
        live_ui.vision_latency_history_us = copyHistory(vision_us_history);
        live_ui.unsafe_detect_history_ms = copyHistory(unsafe_detect_history_ms);
        live_ui.freeze_pipeline_history_ms =
            copyHistory(freeze_pipeline_history_ms);
        live_ui.freeze_cmd_history_ms = copyHistory(freeze_cmd_history_ms);
        live_ui.total_stop_history_ms = copyHistory(total_stop_history_ms);
        live_ui.ack_resume_history_ms = copyHistory(ack_resume_history_ms);
        live_ui.status_rows = {
            {"Vision", safetyStateToString(current_vision_state),
             severityColor(safetyStateToString(current_vision_state))},
            {"Guardian", guardian_state_text, severityColor(guardian_state_text)},
            {"Interlock", interlock_state_text, severityColor(interlock_state_text)},
            {"Routine", routine_label, cv::Scalar(25, 25, 25)},
            {"Pose", current_pose_name, cv::Scalar(25, 25, 25)},
            {"Can arm", frame_is_safe ? "YES" : "NO",
             frame_is_safe ? cv::Scalar(60, 170, 80) : cv::Scalar(60, 60, 200)},
        };
        live_ui.show_focus = true;
        live_ui.focus_label =
            formatFocusScore(focus_score) + " (" + std::string(focus_quality) + ")";
        live_ui.focus_color = focus_color;
        live_ui.focus_fraction = std::clamp(focus_score / 240.0, 0.0, 1.0);
        live_ui.camera_hud_text =
            guardian_armed && guardian->getState() == GuardianState::FROZEN_UNSAFE
                ? "UNSAFE - MOTION HALTED"
                : (guardian_armed && motion_gate_open && decision_is_safe)
                      ? "LIVE - SUPERVISING"
                : (!guardian_armed && frame_is_safe) ? "ARMED - AWAITING START"
                                                     : "INITIALIZING";
        live_ui.camera_hud_color =
            guardian_armed && guardian->getState() == GuardianState::FROZEN_UNSAFE
                ? cv::Scalar(60, 60, 200)
                : (guardian_armed && motion_gate_open ? cv::Scalar(60, 170, 80)
                                                      : cv::Scalar(170, 140, 60));
        live_ui.camera_bottom_left = "CAM:0 · 640x480";
        live_ui.camera_bottom_right = "30fps";
        live_ui.show_frozen_overlay =
            guardian_armed && guardian->getState() == GuardianState::FROZEN_UNSAFE;
        live_ui.frozen_overlay_title = "UNSAFE";
        live_ui.frozen_overlay_subtitle = "MOTION FROZEN - CLEAR WORKSPACE";
        live_ui.show_waiting_overlay =
            guardian_armed && guardian->getState() == GuardianState::RESET_PENDING;
        live_ui.waiting_overlay_text = "WORKSPACE SAFE - PRESS BUTTON TO RESUME";
        live_ui.emphasise_danger =
            guardian_armed && guardian->getState() == GuardianState::FROZEN_UNSAFE;

        cv::Mat camera_frame = display_frame.clone();
        drawCameraOverlay(camera_frame, live_ui);

        const int status_height = std::max(frameHeight(camera_frame), 720);
        if (status_frame.empty() || status_frame.rows != status_height ||
            status_frame.cols != kLiveStatusWidth) {
            status_frame = cv::Mat(status_height,
                                   kLiveStatusWidth,
                                   CV_8UC3,
                                   cv::Scalar(245, 245, 245));
        } else {
            status_frame.setTo(cv::Scalar(245, 245, 245));
        }
        drawStatusDashboard(status_frame, live_ui);

        const int metrics_height = std::max(frameHeight(camera_frame), 560);
        if (metrics_frame.empty() || metrics_frame.rows != metrics_height ||
            metrics_frame.cols != kLiveMetricsWidth) {
            metrics_frame = cv::Mat(metrics_height,
                                    kLiveMetricsWidth,
                                    CV_8UC3,
                                    cv::Scalar(245, 245, 245));
        } else {
            metrics_frame.setTo(cv::Scalar(245, 245, 245));
        }
        drawMetricsDashboard(metrics_frame, live_ui);

        cv::imshow(kLiveCameraWindowName, camera_frame);
        cv::imshow(kLiveStatusWindowName, status_frame);
        cv::imshow(kLiveMetricsWindowName, metrics_frame);
        const int key = cv::waitKey(1);
        if (key == 27) {
            std::cout << "[LIVE_TEST] Exit requested from display window (esc)."
                      << std::endl;
            break;
        }

        int normalized_key = key;
        if (normalized_key >= 0 && normalized_key <= 255) {
            normalized_key = std::tolower(normalized_key);
        }

        if (normalized_key == ' ') {
            if (!requestContinue()) {
                break;
            }
        }

        std::size_t requested_routine_index = 0;
        if (liveRoutineIndexFromKey(normalized_key, requested_routine_index)) {
            if (!selectRoutine(requested_routine_index)) {
                break;
            }
        }

        if (!getLiveRoutineDefinition(selected_routine_index).auto_progress) {
            switch (normalized_key) {
                case 'd':
                    if (!requestManualNudge(-kLiveManualNudgeDegrees, 0, 0, 0,
                                            "BASE LEFT")) {
                        break;
                    }
                    break;
                case 'a':
                    if (!requestManualNudge(kLiveManualNudgeDegrees, 0, 0, 0,
                                            "BASE RIGHT")) {
                        break;
                    }
                    break;
                case 'w':
                    if (!requestManualNudge(0, 0, kLiveManualNudgeDegrees, 0,
                                            "FORWARD")) {
                        break;
                    }
                    break;
                case 's':
                    if (!requestManualNudge(0, 0, -kLiveManualNudgeDegrees, 0,
                                            "BACKWARD")) {
                        break;
                    }
                    break;
                case 'i':
                    if (!requestManualNudge(0, kLiveManualNudgeDegrees, 0, 0, "UP")) {
                        break;
                    }
                    break;
                case 'k':
                    if (!requestManualNudge(0, -kLiveManualNudgeDegrees, 0, 0,
                                            "DOWN")) {
                        break;
                    }
                    break;
                case 'l':
                    if (!requestManualNudge(0, 0, 0, kLiveManualNudgeDegrees, "OPEN")) {
                        break;
                    }
                    break;
                case 'j':
                    if (!requestManualNudge(0, 0, 0, -kLiveManualNudgeDegrees,
                                            "CLOSE")) {
                        break;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    capture_stop_requested.store(true, std::memory_order_relaxed);
    if (capture_thread.joinable()) {
        capture_thread.join();
    }

    cv::destroyWindow(kLiveCameraWindowName);
    cv::destroyWindow(kLiveStatusWindowName);
    stopLiveStepTimer();
    motion_controller_.shutdown();

    if (!processed_any_frame) {
        std::cerr << "[LIVE_TEST] No frames processed. Check camera availability."
                  << std::endl;
        return 1;
    }

    if (motion_faulted) {
        return 1;
    }

    std::cerr << "[LIVE_TEST] Frame stream ended." << std::endl;
    return 0;
}
