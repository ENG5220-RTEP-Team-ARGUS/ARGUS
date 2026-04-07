#pragma once

#include <atomic>
#include <cstdint>

enum class ControlEvent {
    FREEZE_NOW,
    ALLOW_MOTION
};

enum class InterlockState {
    SAFE,
    FROZEN,
    FAULT
};

enum class FreezeReason {
    NONE,
    MARKER_LOST,
    MARKER_OUT_OF_ROI,
    VISION_TIMEOUT,
    POSITION_ERROR,
    WATCHDOG_TIMEOUT,
    UNKNOWN_FAULT
};

class RobotHardware {
public:
    virtual ~RobotHardware() = default;

    // Must immediately stop robot arm and cutter.
    virtual bool freezeMotion() noexcept = 0;

    // Re-enable motion only after ack and safe conditions.
    virtual bool enableMotion() noexcept = 0;
};

class RobotInterlock {
public:
    explicit RobotInterlock(RobotHardware& hardware) noexcept;

    void onControlEvent(
        ControlEvent event,
        FreezeReason reason = FreezeReason::UNKNOWN_FAULT) noexcept;
    void guardianHeartbeat(std::uint32_t tick) noexcept;
    void watchdogCheck(
        std::uint32_t now,
        std::uint32_t max_allowed_delay) noexcept;
    void operatorAcknowledge() noexcept;

    bool motionAllowed() const noexcept;
    FreezeReason freezeReason() const noexcept;
    InterlockState state() const noexcept;

private:
    void freeze(FreezeReason reason) noexcept;
    void attemptResume() noexcept;
    void enterFault(FreezeReason reason) noexcept;

    RobotHardware& hardware_;
    std::atomic<InterlockState> state_;
    std::atomic<bool> operator_ack_;
    std::atomic<FreezeReason> freeze_reason_;
    std::atomic<std::uint32_t> last_guardian_tick_;
};

class PhysicalRobotHardware final : public RobotHardware {
public:
    bool freezeMotion() noexcept override;
    bool enableMotion() noexcept override;

private:
    static void GPIO_WritePin(int pin, bool value) noexcept;

    static constexpr int ARM_ENABLE_PIN = 1;
    static constexpr int CUTTER_ENABLE_PIN = 2;
};
