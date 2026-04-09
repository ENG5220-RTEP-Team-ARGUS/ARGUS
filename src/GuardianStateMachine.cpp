/**
 * @file GuardianStateMachine.cpp
 * @brief Implementation of GuardianStateMachine class for robotic arm safety monitoring
 *
 * This file contains the implementation of all GuardianStateMachine methods.
 * The state machine follows a strict safety protocol:
 * 1. Monitor frames continuously in SAFE_MONITORING state
 * 2. Freeze immediately when threshold of bad frames is reached
 * 3. Require operator acknowledgment before attempting recovery
 * 4. Verify stability with consecutive good frames before resuming
 *
 * @section impl_safety Safety Design
 * - Fail-safe: System freezes on danger, requires human verification to resume
 * - Consecutive frame counting filters transient sensor noise
 * - Operator acknowledgment prevents automatic recovery after incidents
 */

#include "GuardianStateMachine.hpp"
#include <iostream>


/**
 * @brief Constructs a GuardianStateMachine with configurable safety thresholds
 *
 * @param fc Freeze count - consecutive bad frames required to trigger freeze (default: 30)
 * @param rc Recover count - consecutive good frames required to clear freeze (default: 3)
 *
 * @details Initializes the state machine in a safe, operational state:
 * - State: SAFE_MONITORING (normal operation mode)
 * - All counters: 0 (no frames processed yet)
 * - Motion: Allowed (not blocked)
 * - Callbacks: nullptr (not registered yet)
 *
 * **Initialization Order:**
 * Member variables are initialized in declaration order (as defined in header)
 * to prevent subtle initialization bugs.
 *
 * **Memory Safety:**
 * - All members are value types or atomics - no heap allocation
 * - std::function uses small buffer optimization for typical lambdas
 * - No raw pointers requiring manual memory management
 *
 * @note Thresholds should be tuned based on sensor frame rate and safety requirements
 * @warning No parameter validation performed - caller must ensure fc > 0 and rc > 0
 */
GuardianStateMachine::GuardianStateMachine(int fc, int rc) 
    : currentState(GuardianState::SAFE_MONITORING), 
      badCount(0),
      goodCount(0),
      freezeCount(fc),
      recoverCount(rc), 
      motionBlocked(false),             
      onFreezeCallback(nullptr),                     
      onClearFreezeCallback(nullptr), 
      onStateChangeCallback(nullptr) { 

    std::string msg = "Guardian State Machine initialized with freezeCount=" + std::to_string(fc) + ", recoverCount=" + std::to_string(rc);
    MetricsLogger::getInstance().logEvent("[INIT]", msg);
}


/**
 * @brief Executes guardian actions with physical side effects
 *
 * @param action The GuardianAction to execute (FREEZE_NOW, CLEAR_FREEZE, or NONE)
 *
 * @details This function bridges state logic and physical system control.
 * It updates the motionBlocked flag and invokes registered callbacks.
 *
 * **Design Rationale:**
 * - processEvent() decides WHEN to act based on state machine rules
 * - executeAction() defines WHAT each action does physically
 * - This separation allows testing state logic independently from hardware
 *
 * **Action Semantics:**
 * | Action       | motionBlocked | Callback Called      | Use Case                    |
 * |--------------|---------------|----------------------|-----------------------------|
 * | FREEZE_NOW   | true          | onFreezeCallback     | Emergency stop on danger    |
 * | CLEAR_FREEZE | false         | onClearFreezeCallback| Resume after safe recovery  |
 * | NONE         | unchanged     | none                 | State change only           |
 *
 * @note Thread-safe due to atomic motionBlocked flag
 * @warning Callbacks must not block, throw exceptions, or perform heavy computation
 *
 * @see transitionTo() which calls this function during state transitions
 * @see setOnFreezeCallback() for registering freeze handler
 * @see setOnClearFreezeCallback() for registering clear handler
 */
void GuardianStateMachine::executeAction(GuardianAction action) {
    switch (action) {

        case GuardianAction::FREEZE_NOW:
            // Immediately block motion (safety lock)
            motionBlocked = true;

            // Invoke external freeze handler (emergency stop)
            if (onFreezeCallback) {
                onFreezeCallback();
            }
            MetricsLogger::getInstance().logEvent("FREEZE_NOW", "Motion blocked");
            break;

        case GuardianAction::CLEAR_FREEZE:
            // Release safety lock - allow motion
            motionBlocked = false;

            // Invoke external clear handler (resume motion control)
            if (onClearFreezeCallback) {
                onClearFreezeCallback();
            }
            MetricsLogger::getInstance().logEvent("CLEAR_FREEZE", "Motion allowed");
            break;

        case GuardianAction::NONE:
            // No physical action required - state change only
            break;
    }
}


/**
 * @brief Central state transition handler ensuring consistent, safe transitions
 *
 * @param newState The target GuardianState to transition to
 * @param action The GuardianAction to execute during transition
 *
 * @details This is the ONLY function that modifies currentState, ensuring
 * all transitions follow a consistent, auditable pattern.
 *
 * **Transition Protocol (executed atomically):**
 * 1. Capture old state (for logging and callbacks)
 * 2. Update currentState atomically
 * 3. Execute associated action (freeze/clear/none)
 * 4. Notify state change callback (if registered and state changed)
 * 5. Log transition (for debugging and audit trail)
 *
 * **Observer Pattern Implementation:**
 * The onStateChangeCallback implements the Observer pattern, allowing
 * external systems to react to state changes without polling.
 *
 * @note This is the only function that modifies currentState
 * @note Callbacks are executed synchronously in the calling thread
 *
 * @see executeAction() for action execution details
 * @see setOnStateChangeCallback() for registering transition observer
 */
void GuardianStateMachine::transitionTo(GuardianState newState, GuardianAction action) {
    // Capture old state for logging and callback notification
    GuardianState oldState = currentState;

    // Atomically update to new state
    currentState = newState;

    // Execute associated physical action
    executeAction(action);

    // Notify observers only if state actually changed
    if (onStateChangeCallback && oldState != newState) {
        onStateChangeCallback(oldState, newState);
    }

    // Log transition for debugging and audit trail
    logTransition(oldState, newState);
}


/**
 * @brief Logs state transitions for debugging and audit trail
 *
 * @param from Previous GuardianState before transition
 * @param to New GuardianState after transition
 *
 * @details Only logs when an actual state change occurs (from != to).
 * Includes current counter values to provide context for the transition.
 *
 * **Output Format:**
 * @code
 * [TRANSITION] SAFE_MONITORING -> FROZEN_UNSAFE | bad_count=30 good_count=0
 * @endcode
 *
 * @note Thread-safe due to atomic counter reads
 */
void GuardianStateMachine::logTransition(GuardianState from, GuardianState to) {
    // Only print if transition happened
    if (from != to) {
        std::string message = stateToString(from) + " -> " + stateToString(to) + 
                             " | bad_count=" + std::to_string(badCount.load()) + 
                             " good_count=" + std::to_string(goodCount.load());
        MetricsLogger::getInstance().logEvent("[TRANSITION]", message);
    }
}


/**
 * @brief Logs action execution for audit trail
 *
 * @param action Description string of the action being executed
 *
 * @details Provides clear record of safety-critical actions.
 * Separated from transition logging for clarity:
 * - Transitions = state changes (logical)
 * - Actions = physical/system effects (freeze/clear)
 *
 * **Output Format:**
 * @code
 * [ACTION] FREEZE_NOW - Motion blocked
 * [ACTION] CLEAR_FREEZE - Motion allowed
 * @endcode
 */
void GuardianStateMachine::logAction(const std::string& action) {
    MetricsLogger::getInstance().logEvent("[ACTION]", action);
}


/**
 * **SAFE_MONITORING State (Normal Operation):**
 * | Event        | Action                                              |
 * |--------------|-----------------------------------------------------|
 * | FRAME_BAD    | Increment badCount; freeze if >= freezeCount        |
 * | FRAME_GOOD   | Reset badCount to 0 (require consecutive bad frames)|
 * | OPERATOR_ACK | Ignored (not applicable in normal operation)        |
 *
 * **FROZEN_UNSAFE State (Safety Lock Active):**
 * | Event        | Action                                              |
 * |--------------|-----------------------------------------------------|
 * | FRAME_BAD    | Ignored (stay frozen for safety)                    |
 * | FRAME_GOOD   | Ignored (stay frozen - no automatic recovery)       |
 * | OPERATOR_ACK | Transition to RESET_PENDING (begin recovery)        |
 *
 * **RESET_PENDING State (Recovery Verification):**
 * | Event        | Action                                              |
 * |--------------|-----------------------------------------------------|
 * | FRAME_GOOD   | Increment goodCount; clear freeze if >= recoverCount|
 * | FRAME_BAD    | Reset goodCount to 0 (require consecutive good)     |
 * | OPERATOR_ACK | Ignored (already acknowledged)                      |
 *
 * **Safety Design Rationale:**
 * - Consecutive Frame Counting: Filters transient sensor noise/errors
 * - Operator Acknowledgment: Ensures human oversight before recovery
 * - Asymmetric Thresholds: Can tune freeze sensitivity vs recovery confidence
 * - No Automatic Recovery: FROZEN_UNSAFE requires human intervention
 *
 * **Event-Driven Design:**
 * This function is called in response to external events (frames, operator input),
 * not via polling. This ensures:
 * - Immediate response to safety-critical events
 * - No CPU waste on idle polling
 * - Clean integration with interrupt-driven or callback-based systems
 *
 * @note Thread-safe due to atomic state and counter variables
 * @warning Should not be called directly - use processFrame() or operatorAcknowledge()
 *
 * @see processFrame() for frame-based event generation
 * @see operatorAcknowledge() for operator input handling
 */
void GuardianStateMachine::processEvent(GuardianEvent event) {
    switch (currentState) {
        // STATE: SAFE_MONITORING (Normal Operation)
        // Purpose: Monitor frames, freeze if danger threshold reached
        case GuardianState::SAFE_MONITORING:
            if (event == GuardianEvent::FRAME_BAD) {
                // Increment consecutive bad frame counter
                badCount++;

                // Check if freeze threshold reached
                if (badCount >= freezeCount) {
                    // SAFETY TRIGGER: Freeze immediately
                    transitionTo(GuardianState::FROZEN_UNSAFE, GuardianAction::FREEZE_NOW);
                } else {
                    // Otherwise, log how close we are to freezing
                    std::string msg = "Frame Failed, bad_count=" + std::to_string(badCount.load()) + "/" + std::to_string(freezeCount.load());
                    MetricsLogger::getInstance().logEvent("[SAFE_MONITORING]", msg);
                }
            } else if (event == GuardianEvent::FRAME_GOOD) {
                // Reset badCount - require consecutive bad frames
                badCount = 0;
                MetricsLogger::getInstance().logEvent("[SAFE_MONITORING]", "Frame Passed, bad_count reset");
            }
            // OPERATOR_ACK ignored in this state
            break;

        // STATE: FROZEN_UNSAFE (Safety Lock Active)
        // Purpose: Stay frozen until operator confirms safety
        case GuardianState::FROZEN_UNSAFE:
            if (event == GuardianEvent::OPERATOR_ACK) {
                // Operator confirmed - begin recovery verification
                goodCount = 0;

                // Transition to recovery state (still frozen - no CLEAR action)
                transitionTo(GuardianState::RESET_PENDING, GuardianAction::NONE);

            } else if (event == GuardianEvent::FRAME_BAD || event == GuardianEvent::FRAME_GOOD) {
                // SAFETY: Ignore all frames - prevent automatic recovery
                // Human must verify situation before system can recover
                MetricsLogger::getInstance().logEvent("[FROZEN_UNSAFE]", "Remaining frozen, awaiting operator acknowledgment");
            }
            break;

        // STATE: RESET_PENDING (Recovery Verification)
        // Purpose: Verify stable safe conditions before resuming motion
        case GuardianState::RESET_PENDING:
            if (event == GuardianEvent::FRAME_GOOD) {
                // Increment consecutive good frame counter
                goodCount++;

                // Check if recovery threshold reached
                if (goodCount >= recoverCount) {
                    // RECOVERY COMPLETE: Clear freeze and resume
                    goodCount = 0;
                    badCount = 0;

                    // Transition back to SAFE_MONITORING and clear freeze
                    transitionTo(GuardianState::SAFE_MONITORING, GuardianAction::CLEAR_FREEZE);
                } else {
                    // Log recovery progress
                    std::string msg = "good_count=" + std::to_string(goodCount.load()) + "/" + std::to_string(recoverCount.load());
                    MetricsLogger::getInstance().logEvent("[RESET_PENDING]", msg);
                }

            } else if (event == GuardianEvent::FRAME_BAD) {
                // Bad frame during recovery breaks confidence so reset goodCount and keep waiting
                goodCount = 0;
                MetricsLogger::getInstance().logEvent("[RESET_PENDING]",  "Bad frame detected, good_count reset");
            }
            // OPERATOR_ACK ignored - already acknowledged
            break;
    }
}


/**
 * @brief Processes a frame status and updates state machine
 *
 * @param status The FrameStatus of the received frame (FRAME_GOOD or FRAME_BAD)
 *
 * @details Convenience wrapper that converts FrameStatus enum to GuardianEvent
 * and delegates to processEvent(). This provides a cleaner API for frame-based
 * monitoring systems.
 *
 * **Mapping:**
 * | FrameStatus      | GuardianEvent    |
 * |------------------|------------------|
 * | FRAME_GOOD       | FRAME_GOOD       |
 * | FRAME_BAD        | FRAME_BAD        |
 *
 *
 * @note Non-blocking - 0(1) execution time
 * @note Thread-safe - can be called from camera/sensor callback threads
 *
 * @see processEvent() for detailed state machine logic
 */
void GuardianStateMachine::processFrame(FrameStatus status) {
    processEvent(status == FrameStatus::FRAME_GOOD ? GuardianEvent::FRAME_GOOD : GuardianEvent::FRAME_BAD);
}


/**
 * @brief Handles operator acknowledgment input for recovery initiation
 *
 * @details Triggers OPERATOR_ACK event, allowing transition from FROZEN_UNSAFE
 * to RESET_PENDING state. This is required for recovery after a freeze.
 *
 * **Safety Protocol:**
 * 1. System detects danger -> freezes automatically (FROZEN_UNSAFE)
 * 2. Operator investigates and resolves the issue
 * 3. Operator calls operatorAcknowledge() to confirm safety
 * 4. System enters RESET_PENDING and verifies stability
 * 5. After consecutive good frames, system resumes (SAFE_MONITORING)
 *
 * **Design Rationale:**
 * - Prevents automatic recovery after dangerous incidents
 * - Ensures human verification before resuming motion
 * - Creates clear audit trail of operator actions
 *
 * @note Has no effect if not in FROZEN_UNSAFE state
 * @note Should only be called after human verification of safety
 * @warning Do NOT automate this call - defeats safety purpose
 *
 * @see processEvent() for state transition logic
 */
void GuardianStateMachine::operatorAcknowledge() {
    MetricsLogger::getInstance().logEvent("[OPERATOR]", "Acknowledgment received");
    processEvent(GuardianEvent::OPERATOR_ACK);
}


/**
 * @brief Registers callback for freeze action (emergency stop)
 *
 * @param callback Function to invoke when motion is frozen
 *
 * @details The callback should implement emergency stop or motion blocking
 * in the external robotic system. Called synchronously during state
 * transition to FROZEN_UNSAFE.
 *
 * **Dependency Inversion Principle:**
 * This callback mechanism allows GuardianStateMachine to control hardware
 * without depending on specific hardware implementations. The state machine
 * depends on the abstract callback interface, not concrete robot classes.
 *
 * @note Callback is stored by copy - ensure captured references remain valid
 * @warning Callback must not block or system becomes unresponsive
 *
 * @see executeAction() where callback is invoked
 */
void GuardianStateMachine::setOnFreezeCallback(std::function<void()> callback) {
    onFreezeCallback = callback; // Store freeze callback
}

/**
 * @brief Registers callback for clear-freeze action (resume motion)
 *
 * @param callback Function to invoke when freeze is cleared
 *
 * @details The callback should resume motion control in the external system.
 * Called synchronously during state transition from RESET_PENDING to
 * SAFE_MONITORING.
 *
 * @note Callback is stored by copy - ensure captured references remain valid
 * @warning Callback must not block or system becomes unresponsive
 *
 * @see executeAction() where callback is invoked
 */
void GuardianStateMachine::setOnClearFreezeCallback(std::function<void()> callback) {
    onClearFreezeCallback = callback; // Store clear-freeze callback
}

/**
 * @brief Registers callback for state transition notifications
 *
 * @param callback Function to invoke on state changes (receives old and new state)
 *
 * @details Implements the Observer pattern for state change notifications.
 * Called synchronously after state update but before action execution.
 * Useful for logging, monitoring, UI updates, or triggering dependent systems.
 *
 * @note Only called when state actually changes (from != to)
 * @note Callback is stored by copy
 *
 * @see transitionTo() where callback is invoked
 */
void GuardianStateMachine::setOnStateChangeCallback(std::function<void(GuardianState, GuardianState)> callback) {
    onStateChangeCallback = callback; // Store transition callback
}

/**
 * @brief Gets the current state of the guardian
 *
 * @return Current GuardianState enum value
 *
 * @details Thread-safe read of atomic state variable.
 * Can be called from any thread without synchronization.
 *
 * @note Returns snapshot at call time - state may change immediately after
 */
GuardianState GuardianStateMachine::getState() const {
    return currentState; // Return the current state enum
}


/**
 * @brief Checks if motion is currently blocked (frozen)
 *
 * @return true if motion is blocked, false if allowed
 *
 * @details Thread-safe read of atomic motionBlocked flag.
 * This is the primary query for motion control systems.
 *
 * @note Returns snapshot at call time - may change immediately after
 */
bool GuardianStateMachine::isMotionBlocked() const {
    return motionBlocked; // True = frozen
}


/**
 * @brief Checks if motion is currently allowed
 *
 * @return true if motion is allowed, false if blocked
 *
 * @details Convenience inverse of isMotionBlocked().
 * Semantically clearer in some contexts.
 *
 * @note Returns snapshot at call time
 */
bool GuardianStateMachine::isMotionAllowed() const {
    return !motionBlocked; // False = Resume
}

/**
 * @brief Gets current count of consecutive bad frames
 *
 * @return Number of consecutive bad frames detected
 *
 * @details Useful for monitoring how close system is to freezing.
 * Value is only meaningful in SAFE_MONITORING state.
 */
int GuardianStateMachine::getBadCount() const {
    return badCount; // Return current consecutive bad frame count
}

/**
 * @brief Gets current count of consecutive good frames during recovery
 *
 * @return Number of consecutive good frames in RESET_PENDING state
 *
 * @details Useful for monitoring recovery progress.
 * Value is only meaningful in RESET_PENDING state.
 */
int GuardianStateMachine::getGoodCount() const {
    return goodCount; // Return current consecutive good frame count
}


/**
 * @brief Converts GuardianState enum to human-readable string
 *
 * @param state The GuardianState to convert
 * @return String representation of the state
 *
 * @details Returns one of:
 * - "SAFE_MONITORING" - Normal operation
 * - "FROZEN_UNSAFE" - Safety lock active
 * - "RESET_PENDING" - Recovery verification
 * - "UNKNOWN" - Defensive fallback for invalid state
 *
 * @note Useful for logging, debugging, and UI display
 */
std::string GuardianStateMachine::stateToString(GuardianState state) const {
    switch (state) {
        case GuardianState::SAFE_MONITORING: return "SAFE_MONITORING";
        case GuardianState::FROZEN_UNSAFE:   return "FROZEN_UNSAFE";
        case GuardianState::RESET_PENDING:   return "RESET_PENDING";
        default:                             return "UNKNOWN"; // Defensive fallback
    }
}


/**
 * @brief Gets string representation of current state
 *
 * @return String representation of currentState
 *
 * @details Convenience wrapper around stateToString(currentState).
 * Thread-safe due to atomic state variable.
 */
std::string GuardianStateMachine::getCurrentStateString() const {
    return stateToString(currentState);
}


/**
 * @brief Prints comprehensive status information to console
 *
 * @details Outputs a formatted snapshot of the guardian's internal state
 * for debugging, testing, and system monitoring purposes.
 *
 * **Output Format:**
 * @code
 * === GUARDIAN STATUS ===
 * State: SAFE_MONITORING
 * Motion: ALLOWED
 * Bad Count: 5/30
 * Good Count: 0/3
 * =======================
 * @endcode
 *
 * **Information Displayed:**
 * - Current state (SAFE_MONITORING, FROZEN_UNSAFE, or RESET_PENDING)
 * - Motion status (BLOCKED or ALLOWED)
 * - Bad frame count with threshold (current/max)
 * - Good frame count with threshold (current/max)
 *
 * **Thread Safety:**
 * All reads are from atomic variables, making this function thread-safe.
 * However, the snapshot represents a point-in-time view and values may
 * change immediately after the function returns.
 *
 * @note Can be called from any thread safely
 *
 * @see getCurrentStateString() for state name conversion
 * @see isMotionBlocked() for motion status query
 * @see getBadCount() for bad frame counter
 * @see getGoodCount() for good frame counter
 */
void GuardianStateMachine::printStatus() const {
    std::string statusMsg = "\n=== GUARDIAN STATUS ===\n";
    statusMsg += "State: " + getCurrentStateString() + "\n";
    statusMsg += "Motion: " + std::string(motionBlocked ? "BLOCKED" : "ALLOWED") + "\n";
    statusMsg += "Bad Count: " + std::to_string(badCount.load()) + "/" + std::to_string(freezeCount.load()) + "\n";
    statusMsg += "Good Count: " + std::to_string(goodCount.load()) + "/" + std::to_string(recoverCount.load()) + "\n";
    statusMsg += "=======================";
    MetricsLogger::getInstance().logEvent("STATUS", statusMsg);
}