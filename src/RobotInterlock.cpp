#include "RobotInterlock.hpp"

RobotInterlock::RobotInterlock(RobotHardware& hardware) noexcept
    : hardware_(hardware),
      state_(InterlockState::SAFE),
      operator_ack_(false),
      freeze_reason_(FreezeReason::NONE),
      last_guardian_tick_(0) {}

void RobotInterlock::onControlEvent(
    ControlEvent event,
    FreezeReason reason) noexcept {
    if (event == ControlEvent::FREEZE_NOW) {
        freeze(reason);
    } else {
        attemptResume();
    }
}

void RobotInterlock::guardianHeartbeat(std::uint32_t tick) noexcept {
    last_guardian_tick_.store(tick, std::memory_order_relaxed);
}

void RobotInterlock::watchdogCheck(
    std::uint32_t now,
    std::uint32_t max_allowed_delay) noexcept {
    const std::uint32_t last =
        last_guardian_tick_.load(std::memory_order_relaxed);

    if ((now - last) > max_allowed_delay) {
        freeze(FreezeReason::WATCHDOG_TIMEOUT);
    }
}

void RobotInterlock::operatorAcknowledge() noexcept {
    if (state_.load(std::memory_order_relaxed) == InterlockState::FROZEN) {
        operator_ack_.store(true, std::memory_order_relaxed);
    }
}

bool RobotInterlock::motionAllowed() const noexcept {
    return state_.load(std::memory_order_relaxed) == InterlockState::SAFE;
}

FreezeReason RobotInterlock::freezeReason() const noexcept {
    return freeze_reason_.load(std::memory_order_relaxed);
}

InterlockState RobotInterlock::state() const noexcept {
    return state_.load(std::memory_order_relaxed);
}

void RobotInterlock::freeze(FreezeReason reason) noexcept {
    InterlockState expected = InterlockState::SAFE;
    if (state_.compare_exchange_strong(
            expected,
            InterlockState::FROZEN,
            std::memory_order_acq_rel)) {
        freeze_reason_.store(reason, std::memory_order_relaxed);
        operator_ack_.store(false, std::memory_order_relaxed);
        hardware_.freezeMotion();
    }
}

void RobotInterlock::attemptResume() noexcept {
    if (state_.load(std::memory_order_relaxed) != InterlockState::FROZEN) {
        return;
    }

    if (!operator_ack_.load(std::memory_order_relaxed)) {
        return;
    }

    hardware_.enableMotion();
    freeze_reason_.store(FreezeReason::NONE, std::memory_order_relaxed);
    state_.store(InterlockState::SAFE, std::memory_order_release);
}

void PhysicalRobotHardware::freezeMotion() noexcept {
    GPIO_WritePin(ARM_ENABLE_PIN, false);
    GPIO_WritePin(CUTTER_ENABLE_PIN, false);
}

void PhysicalRobotHardware::enableMotion() noexcept {
    GPIO_WritePin(ARM_ENABLE_PIN, true);
    GPIO_WritePin(CUTTER_ENABLE_PIN, true);
}

void PhysicalRobotHardware::GPIO_WritePin(int pin, bool value) noexcept {
    // Replace with platform-specific GPIO or safety relay API.
    (void)pin;
    (void)value;
}
