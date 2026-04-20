/**
 * @file GuardianStateMachine.hpp
 * @brief Finite state machine for ARGUS safety supervision.
 *
 * Implements a three-state FSM that monitors vision safety and controls
 * motion freeze/recovery. Transitions are event-driven with hysteresis
 * to prevent oscillations. Designed for real-time operation with bounded
 * latency and no dynamic memory allocation.
 */

#ifndef GUARDIAN_STATE_MACHINE_HPP  // To prevent multiple inclusion of this header file
#define GUARDIAN_STATE_MACHINE_HPP  // To define the guard macro (included only once)

#include <string>
#include <functional>


// Enum Definitions

/**
 * @brief States of the Guardian State Machine.
 */
enum class GuardianState {
    SAFE_MONITORING,   ///< Normal monitoring state (system running normally)
    FROZEN_UNSAFE,     ///< Unsafe detected -> motion is frozen
    RESET_PENDING      ///< Operator acknowledged -> waiting for stable recovery
};

/**
 * @brief Status of individual vision frames.
 */
enum class FrameStatus {
    FRAME_GOOD,        ///< Frame is in safe condition
    FRAME_BAD          ///< Frame indicates unsafe condition
};

/**
 * @brief Events that trigger state transitions.
 */
enum class GuardianEvent {
    FRAME_GOOD,        ///< Event triggered when a safe frame is received
    FRAME_BAD,         ///< Event triggered when an unsafe frame is received
    OPERATOR_ACK       ///< Event triggered when operator acknowledges/reset
};

/**
 * @brief Actions taken in response to events.
 */
enum class GuardianAction {
    NONE,              ///< No action required
    FREEZE_NOW,        ///< Immediately freeze/block motion
    CLEAR_FREEZE       ///< Clear freeze and resume motion
};

/**
 * @brief Guardian State Machine for ARGUS safety supervision.
 *
 * A three-state FSM that monitors vision safety and controls motion freeze/recovery.
 * Transitions are event-driven with hysteresis to prevent oscillations. Designed for
 * real-time operation with bounded latency and no dynamic memory allocation.
 *
 * State transitions:
 * - SAFE_MONITORING: Normal operation, monitoring for unsafe frames
 * - FROZEN_UNSAFE: Motion frozen due to consecutive unsafe frames
 * - RESET_PENDING: Waiting for operator ack after safety recovery
 *
 * @note Thread-safe for single-writer, multiple-reader access patterns.
 */
class GuardianStateMachine {
public:
    /**
     * @brief Constructs the Guardian State Machine with hysteresis thresholds.
     *
     * @param freezeThreshold Number of consecutive bad frames to trigger freeze
     * @param recoverThreshold Number of consecutive good frames to allow recovery
     */
    GuardianStateMachine(int freezeThreshold = 15, int recoverThreshold = 10);

    /**
     * @brief Processes a vision frame and returns the required action.
     *
     * Updates internal state based on frame status and returns any action
     * that should be taken (freeze, clear freeze, or none).
     *
     * @param frameStatus Status of the current vision frame
     * @return Action to take based on state transition
     */
    GuardianAction processFrame(FrameStatus frameStatus);

    /**
     * @brief Handles operator acknowledgment event.
     *
     * Should be called when the operator acknowledges a freeze event.
     * Transitions from RESET_PENDING back to SAFE_MONITORING.
     *
     * @return Action to take (typically CLEAR_FREEZE)
     */
    GuardianAction acknowledgeReset();

    /**
     * @brief Gets the current state of the FSM.
     * @return Current GuardianState
     */
    GuardianState getCurrentState() const;

    /**
     * @brief Gets a string representation of the current state.
     * @return State name as string
     */
    std::string getCurrentStateString() const;

private:

    // Current state of the state machine
    GuardianState currentState;

    // Counts consecutive bad frames (used in SAFE_MONITORING)
    int badCount;

    // Counts consecutive good frames (used in RESET_PENDING)
    int goodCount;

    // Threshold: number of bad frames required to freeze
    int freezeCount;

    // Threshold: number of good frames required to clear freeze
    int recoverCount;

    // Indicates whether motion is currently blocked
    bool motionBlocked;
    
    // Callback function executed when freeze occurs
    std::function<void()> onFreezeCallback;

    // Callback function executed when freeze is cleared
    std::function<void()> onClearFreezeCallback;

    // Callback function executed when state changes
    std::function<void(GuardianState, GuardianState)> onStateChangeCallback;

    // Executes an action (FREEZE_NOW or CLEAR_FREEZE)
    void executeAction(GuardianAction action);

    // Handles transition from one state to another
    void transitionTo(GuardianState newState, GuardianAction action);

    // Logs state transition
    void logTransition(GuardianState from, GuardianState to);

    // Logs actions
    void logAction(const std::string& action);

public:

    // Constructor with configurable thresholds:
    // fc = number of bad frames to trigger freeze
    // rc = number of good frames to clear freeze
    GuardianStateMachine(int fc = 30, int rc = 3);    // Note: To be adjusted accordingly in future revisions
    
    // Main function that processes incoming events
    void processEvent(GuardianEvent event);

    // Convenience function to convert FrameStatus into GuardianEvent
    void processFrame(FrameStatus status);

    // Function to handle operator acknowledgment input
    void operatorAcknowledge();
    
    // Set callback for freeze action
    void setOnFreezeCallback(std::function<void()> callback);

    // Set callback for clear-freeze action
    void setOnClearFreezeCallback(std::function<void()> callback);

    // Set callback for state transitions
    void setOnStateChangeCallback(
        std::function<void(GuardianState, GuardianState)> callback);
    
    // Returns the current state
    GuardianState getState() const;

    // Returns true if motion is currently blocked
    bool isMotionBlocked() const;

    // Returns true if motion is currently allowed
    bool isMotionAllowed() const;

    // Returns number of consecutive bad frames
    int getBadCount() const;

    // Returns number of consecutive good frames
    int getGoodCount() const;
    
    // Converts state enum into readable string
    std::string stateToString(GuardianState state) const;

    // Returns string version of current state
    std::string getCurrentStateString() const;

    // Prints full status of the guardian
    void printStatus() const;
};

#endif // GUARDIAN_STATE_MACHINE_HPP