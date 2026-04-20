/**
 * @file RobotInterlock.hpp
 * @brief Safety interlock gate between guardian decisions and robot motion.
 */

#pragma once

#include <atomic>
#include <cstdint>

/**
 * @brief Control commands issued to the interlock.
 */
enum class ControlEvent {
    FREEZE_NOW,  ///< Immediately freeze/deny motion.
    ALLOW_MOTION  ///< Attempt to allow motion when safety conditions permit.
};

/**
 * @brief Interlock state machine output states.
 */
enum class InterlockState {
    SAFE,  ///< Motion allowed.
    FROZEN,  ///< Motion blocked due to safety freeze.
    FAULT  ///< Motion blocked due to hardware/path fault.
};

/**
 * @brief Latched freeze/fault reason.
 */
enum class FreezeReason {
    NONE,  ///< No freeze reason currently latched.
    MARKER_LOST,  ///< Vision lost expected marker.
    MARKER_OUT_OF_ROI,  ///< Marker/tool moved outside allowed ROI.
    VISION_TIMEOUT,  ///< Vision heartbeat/timing failure.
    POSITION_ERROR,  ///< Position-related safety violation.
    DEPTH_EXCEEDED,  ///< Forbidden depth-layer/colour detected.
    WATCHDOG_TIMEOUT,  ///< Guardian heartbeat timeout.
    UNKNOWN_FAULT  ///< Unspecified fault reason.
};

/**
 * @brief Abstract hardware contract used by RobotInterlock.
 */
class RobotHardware {
public:
    virtual ~RobotHardware() = default;

    /**
     * @brief Immediately stop motion output.
     * @return `true` on success, otherwise `false`.
     */
    virtual bool freezeMotion() noexcept = 0;

    /**
     * @brief Re-enable motion output after safe/ack conditions.
     * @return `true` on success, otherwise `false`.
     */
    virtual bool enableMotion() noexcept = 0;
};

/**
 * @brief Thread-safe safety gate that owns interlock state transitions.
 */
class RobotInterlock {
public:
    /**
     * @brief Construct interlock bound to a hardware implementation.
     * @param hardware Hardware adapter used for freeze/enable side effects.
     */
    explicit RobotInterlock(RobotHardware& hardware) noexcept;

    /**
     * @brief Submit a control event to the interlock state machine.
     * @param event Control event (`FREEZE_NOW` or `ALLOW_MOTION`).
     * @param reason Associated freeze reason when applicable.
     */
    void onControlEvent(
        ControlEvent event,
        FreezeReason reason = FreezeReason::UNKNOWN_FAULT) noexcept;

    /**
     * @brief Update guardian heartbeat tick observed by interlock.
     * @param tick Monotonic guardian tick counter.
     */
    void guardianHeartbeat(std::uint32_t tick) noexcept;

    /**
     * @brief Evaluate watchdog timeout against last guardian heartbeat.
     * @param now Current monotonic tick.
     * @param max_allowed_delay Maximum allowed stale-heartbeat delay in ticks.
     */
    void watchdogCheck(
        std::uint32_t now,
        std::uint32_t max_allowed_delay) noexcept;

    /**
     * @brief Register operator acknowledge for motion resume attempts.
     */
    void operatorAcknowledge() noexcept;

    /**
     * @brief Query whether motion is currently allowed.
     * @return `true` when interlock state is SAFE.
     */
    bool motionAllowed() const noexcept;

    /**
     * @brief Get currently latched freeze reason.
     * @return Freeze reason.
     */
    FreezeReason freezeReason() const noexcept;

    /**
     * @brief Get current interlock state.
     * @return Interlock state.
     */
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

/**
 * @brief Basic physical hardware adapter for interlock freeze/enable actions.
 */
class PhysicalRobotHardware final : public RobotHardware {
public:
    /**
     * @brief Apply immediate freeze to physical hardware lines.
     * @return `true` on success, otherwise `false`.
     */
    bool freezeMotion() noexcept override;

    /**
     * @brief Re-enable motion on physical hardware lines.
     * @return `true` on success, otherwise `false`.
     */
    bool enableMotion() noexcept override;

private:
    static void GPIO_WritePin(int pin, bool value) noexcept;

    static constexpr int ARM_ENABLE_PIN = 1;
    static constexpr int CUTTER_ENABLE_PIN = 2;
};
