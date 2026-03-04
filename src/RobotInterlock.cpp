#include <atomic>
#include <cstdint>

/* CONTROL EVENTS */

enum class ControlEvent {
    FREEZE_NOW,
    ALLOW_MOTION
};

/* INTERLOCK STATE */

enum class InterlockState {
    SAFE,        // Motion + cutting allowed
    FROZEN       // Motion + cutting disabled
};

/* FREEZE REASON*/

enum class FreezeReason {
    NONE,
    MARKER_LOST,
    MARKER_OUT_OF_ROI,
    VISION_TIMEOUT,
    POSITION_ERROR,
    WATCHDOG_TIMEOUT,
    UNKNOWN_FAULT
};

/* HARDWARE ABSTRACTION */

class RobotHardware {
public:
    virtual ~RobotHardware() = default;

    // Must immediately stop robot arm AND cutter
    virtual void freezeMotion() noexcept = 0;

    // Re-enable motion only after ack + safe
    virtual void enableMotion() noexcept = 0;
};

/* ROBOT INTERLOCK */

class RobotInterlock {
public:
    explicit RobotInterlock(RobotHardware& hardware) noexcept
        : hardware_(hardware),
          state_(InterlockState::SAFE),
          operator_ack_(false),
          freeze_reason_(FreezeReason::NONE),
          last_guardian_tick_(0)
    {}

    /* Guardian Interface */

    void onControlEvent(ControlEvent event,
                        FreezeReason reason = FreezeReason::UNKNOWN_FAULT) noexcept
    {
        if (event == ControlEvent::FREEZE_NOW) {
            freeze(reason);
        } else { // ALLOW_MOTION
            attemptResume();
        }
    }

    void guardianHeartbeat(uint32_t tick) noexcept {
        last_guardian_tick_.store(tick, std::memory_order_relaxed);
    }

    /* Watchdog Interface */

    // Called from RT watchdog task
    void watchdogCheck(uint32_t now,
                       uint32_t max_allowed_delay) noexcept
    {
        uint32_t last =
            last_guardian_tick_.load(std::memory_order_relaxed);

        if ((now - last) > max_allowed_delay) {
            freeze(FreezeReason::WATCHDOG_TIMEOUT);
        }
    }

    /* Operator Interface */

    void operatorAcknowledge() noexcept {
        if (state_.load(std::memory_order_relaxed)
            == InterlockState::FROZEN) {
            operator_ack_.store(true, std::memory_order_relaxed);
        }
    }

    /* Motion Gating */

    bool motionAllowed() const noexcept {
        return state_.load(std::memory_order_relaxed)
               == InterlockState::SAFE;
    }

    FreezeReason freezeReason() const noexcept {
        return freeze_reason_.load(std::memory_order_relaxed);
    }

    InterlockState state() const noexcept {
        return state_.load(std::memory_order_relaxed);
    }

private:
    /* Internal Logic */

    void freeze(FreezeReason reason) noexcept {
        InterlockState expected = InterlockState::SAFE;
        if (state_.compare_exchange_strong(
                expected,
                InterlockState::FROZEN,
                std::memory_order_acq_rel)) {

            freeze_reason_.store(reason,
                                 std::memory_order_relaxed);
            operator_ack_.store(false,
                                std::memory_order_relaxed);

            hardware_.freezeMotion();   // HARD PHYSICAL STOP
        }
    }

    void attemptResume() noexcept {
        if (state_.load(std::memory_order_relaxed)
            != InterlockState::FROZEN)
            return;

        if (!operator_ack_.load(std::memory_order_relaxed))
            return;

        hardware_.enableMotion();
        freeze_reason_.store(FreezeReason::NONE,
                             std::memory_order_relaxed);
        state_.store(InterlockState::SAFE,
                     std::memory_order_release);
    }

private:
    RobotHardware& hardware_;
    std::atomic<InterlockState> state_;
    std::atomic<bool> operator_ack_;
    std::atomic<FreezeReason> freeze_reason_;
    std::atomic<uint32_t> last_guardian_tick_;
};

/* PHYSICAL HARDWARE IMPLEMENTATION */

class PhysicalRobotHardware final : public RobotHardware {
public:
    void freezeMotion() noexcept override {
        // Stop robot arm immediately
        GPIO_WritePin(ARM_ENABLE_PIN, false);

        // Stop cutting tool immediately
        GPIO_WritePin(CUTTER_ENABLE_PIN, false);
    }

    void enableMotion() noexcept override {
        // Re-enable ONLY after ack + safe
        GPIO_WritePin(ARM_ENABLE_PIN, true);
        GPIO_WritePin(CUTTER_ENABLE_PIN, true);
    }

private:
    static void GPIO_WritePin(int pin, bool value) noexcept {
        // Replace with:
        // HAL_GPIO_WritePin(...)
        // direct register write
        // safety relay / STO control
        (void)pin;
        (void)value;
    }

    static constexpr int ARM_ENABLE_PIN    = 1;
    static constexpr int CUTTER_ENABLE_PIN = 2;
};
