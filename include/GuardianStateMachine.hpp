/**
 * @file GuardianStateMachine.hpp
 * @brief Finite-state safety supervisor used by ARGUS.
 *
 * The Guardian is a compact three-state FSM with hysteresis:
 * - unsafe evidence must persist for N bad frames before freeze
 * - safe evidence must persist for M good frames before clear
 *
 * This design avoids oscillation on noisy vision input while keeping
 * behavior deterministic and easy to audit in logs.
 */

#ifndef GUARDIAN_STATE_MACHINE_HPP
#define GUARDIAN_STATE_MACHINE_HPP

#include <functional>
#include <string>

/**
 * @brief Internal FSM states.
 */
enum class GuardianState {
    SAFE_MONITORING,  ///< Normal operation; monitoring incoming frame status.
    FROZEN_UNSAFE,    ///< Unsafe condition latched; motion remains blocked.
    RESET_PENDING     ///< Operator acknowledged; waiting for stable good frames.
};

/**
 * @brief Per-frame safety classification delivered by vision.
 */
enum class FrameStatus {
    FRAME_GOOD,  ///< Frame classified as safe.
    FRAME_BAD    ///< Frame classified as unsafe.
};

/**
 * @brief Events consumed by the FSM transition logic.
 */
enum class GuardianEvent {
    FRAME_GOOD,   ///< Safe-frame event.
    FRAME_BAD,    ///< Unsafe-frame event.
    OPERATOR_ACK  ///< Operator/manual acknowledge event.
};

/**
 * @brief Side effects emitted by transitions.
 */
enum class GuardianAction {
    NONE,         ///< No hardware action required.
    FREEZE_NOW,   ///< Immediate freeze/block action.
    CLEAR_FREEZE  ///< Clear freeze and allow motion.
};

/**
 * @brief Event-driven safety state machine with callback hooks.
 *
 * Integration model:
 * - `processFrame(...)` is called for each vision decision.
 * - `operatorAcknowledge()` is called when operator input is received.
 * - callbacks notify external systems (interlock/motion controller) on
 *   freeze/clear transitions.
 */
class GuardianStateMachine {
public:
    /**
     * @brief Construct the FSM with frame-count hysteresis thresholds.
     * @param fc Consecutive bad frames required to freeze.
     * @param rc Consecutive good frames required to clear after ack.
     */
    GuardianStateMachine(int fc = 30, int rc = 3);

    /**
     * @brief Process an explicit FSM event.
     */
    void processEvent(GuardianEvent event);

    /**
     * @brief Convenience wrapper that maps frame status to an FSM event.
     */
    void processFrame(FrameStatus status);

    /**
     * @brief Inject an operator acknowledge event.
     */
    void operatorAcknowledge();

    /**
     * @brief Register callback fired when freeze is commanded.
     */
    void setOnFreezeCallback(std::function<void()> callback);

    /**
     * @brief Register callback fired when freeze is cleared.
     */
    void setOnClearFreezeCallback(std::function<void()> callback);

    /**
     * @brief Register callback fired on state transitions.
     */
    void setOnStateChangeCallback(
        std::function<void(GuardianState, GuardianState)> callback);

    /**
     * @brief Get current FSM state.
     */
    GuardianState getState() const;

    /**
     * @brief True when guardian currently blocks motion.
     */
    bool isMotionBlocked() const;

    /**
     * @brief True when motion is currently allowed.
     */
    bool isMotionAllowed() const;

    /**
     * @brief Get current consecutive bad-frame count.
     */
    int getBadCount() const;

    /**
     * @brief Get current consecutive good-frame count.
     */
    int getGoodCount() const;

    /**
     * @brief Convert a state enum to a readable string.
     */
    std::string stateToString(GuardianState state) const;

    /**
     * @brief Return readable string for current state.
     */
    std::string getCurrentStateString() const;

    /**
     * @brief Print a status snapshot for debugging.
     */
    void printStatus() const;

private:
    // State and counters
    GuardianState currentState;
    int badCount;
    int goodCount;
    int freezeCount;
    int recoverCount;
    bool motionBlocked;

    // Integration callbacks
    std::function<void()> onFreezeCallback;
    std::function<void()> onClearFreezeCallback;
    std::function<void(GuardianState, GuardianState)> onStateChangeCallback;

    // Internal helpers
    void executeAction(GuardianAction action);
    void transitionTo(GuardianState newState, GuardianAction action);
    void logTransition(GuardianState from, GuardianState to);
    void logAction(const std::string& action);
};

#endif  // GUARDIAN_STATE_MACHINE_HPP
