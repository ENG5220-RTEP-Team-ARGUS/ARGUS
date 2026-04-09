/**
 * @file GuardianStateMachine.hpp
 * @brief Guardian State Machine for robotic arm safety monitoring
 *
 * This header defines a safety state machine that monitors frame conditions
 * and controls motion blocking for robotic arm applications. It implements
 * a three-state system (SAFE_MONITORING, FROZEN_UNSAFE, RESET_PENDING) with
 * configurable thresholds for freeze and recovery conditions.
 *
 * @section design_principles Design Principles
 * - Single Responsibility: Manages only safety state logic and motion control decisions
 * - Open/Closed: Extensible via callbacks without modifying core state machine logic
 * - Liskov Substitution: Can be used polymorphically if needed in future extensions
 * - Interface Segregation: Clean, minimal public interface with only necessary methods
 * - Dependency Inversion: Depends on abstractions (callbacks) not concrete implementations
 *
 * @section thread_safety Thread Safety
 * All state variables use std::atomic for lock-free thread safety.
 * Callbacks are executed synchronously within the calling thread.
 */

#ifndef GUARDIAN_STATE_MACHINE_HPP  // To prevent multiple inclusion of this header file
#define GUARDIAN_STATE_MACHINE_HPP  // To define the guard macro (included only once)

#include <string>
#include <functional>
#include <atomic>
#include "MetricsLogger.hpp"

/**
 * @enum GuardianState
 * @brief Represents the current operational state of the guardian system
 *
 * The state machine follows a strict safety protocol:
 * SAFE_MONITORING → FROZEN_UNSAFE → RESET_PENDING → SAFE_MONITORING
 */
enum class GuardianState {
    SAFE_MONITORING,   ///< Normal monitoring state - system running normally, counting bad frames
    FROZEN_UNSAFE,     ///< Unsafe condition detected - motion is frozen, awaiting operator acknowledgment
    RESET_PENDING      ///< Operator acknowledged - waiting for stable recovery with consecutive good frames
};

/**
 * @enum FrameStatus
 * @brief Represents the safety status of a received frame from sensors
 */
enum class FrameStatus {
    FRAME_GOOD,        ///< Frame indicates safe condition
    FRAME_BAD          ///< Frame indicates unsafe condition
};

/**
 * @enum GuardianEvent
 * @brief Events that trigger state machine transitions
 *
 * These events are processed by the state machine to determine state transitions
 * and actions. Events are typically generated from frame processing or operator input.
 */
enum class GuardianEvent {
    FRAME_GOOD,        ///< Event triggered when a safe frame is received
    FRAME_BAD,         ///< Event triggered when an unsafe frame is received
    OPERATOR_ACK       ///< Event triggered when operator acknowledges/resets the system
};

/**
 * @enum GuardianAction
 * @brief Actions executed during state transitions
 *
 * Actions represent physical effects on the robotic system (motion control).
 * They are executed via registered callbacks to decouple state logic from hardware.
 */
enum class GuardianAction {
    NONE,              ///< No action required (state change only)
    FREEZE_NOW,        ///< Immediately freeze/block motion (emergency stop)
    CLEAR_FREEZE       ///< Clear freeze and resume motion (safety release)
};


/**
 * @class GuardianStateMachine
 * @brief Thread-safe state machine for robotic arm safety monitoring
 *
 * This class implements a safety guardian that monitors incoming frame data
 * and controls motion blocking based on configurable thresholds. It uses
 * atomic operations for thread safety and provides callback mechanisms for
 * external system integration.
 *
 * @section design_rationale Design Rationale
 *
 * **Encapsulation:**
 * - All state variables are private with atomic protection
 * - Public interface provides only necessary operations
 * - Internal counters cannot be modified externally (read-only getters)
 *
 * **Event-Driven Architecture:**
 * - No polling loops - responds to events via processFrame() and operatorAcknowledge()
 * - Callbacks enable non-blocking integration with external systems
 * - Deterministic state transitions with 0(1) complexity
 *
 * **Memory Safety:**
 * - No dynamic memory allocation in critical paths
 * - All data structures are stack-allocated or atomic
 * - Callbacks use std::function
 */
class GuardianStateMachine {
private:
    // ==================== Private Member Variables ====================

    /**
     * @brief Current state of the state machine
     * @details Thread-safe atomic variable to prevent race conditions.
     *          Can be safely read from multiple threads.
     */
    std::atomic<GuardianState> currentState;

    /**
     * @brief Counts consecutive bad frames in SAFE_MONITORING state
     * @details Reset to 0 when a good frame is received.
     *          When this reaches freezeCount, system transitions to FROZEN_UNSAFE.
     */
    std::atomic<int> badCount;

    /**
     * @brief Counts consecutive good frames in RESET_PENDING state
     * @details Reset to 0 when a bad frame is received during recovery.
     *          When this reaches recoverCount, system transitions back to SAFE_MONITORING.
     */
    std::atomic<int> goodCount;

    /**
     * @brief Threshold: number of consecutive bad frames required to trigger freeze
     * @details Configurable via constructor, typically around 30 frames depending on
     *          frame rate and safety requirements.
     *          Higher values = more tolerance for transient errors, 
     *          lower values = faster response to danger.
     */
    std::atomic<int> freezeCount;

    /**
     * @brief Threshold: number of consecutive good frames required to clear freeze
     * @details Configurable via constructor, typically around 10 frames.
     *          Higher values = more confidence required before resuming motion.
     */
    std::atomic<int> recoverCount;

    /**
     * @brief Indicates whether motion is currently blocked
     * @details true = motion blocked (frozen), false = motion allowed.
     *          This flag is the primary output for motion control systems.
     */
    std::atomic<bool> motionBlocked;
    
    /**
     * @brief Callback function executed when freeze action occurs
     * @details Should trigger emergency stop or motion blocking in external system.
     *          Must be non-blocking and thread-safe.
     *          Called synchronously during state transition.
     */
    std::function<void()> onFreezeCallback;

    /**
     * @brief Callback function executed when freeze is cleared
     * @details Should resume motion control in external system.
     *          Must be non-blocking and thread-safe.
     *          Called synchronously during state transition.
     */
    std::function<void()> onClearFreezeCallback;

    /**
     * @brief Callback function executed on state transitions
     * @details Receives old state and new state as parameters.
     *          Useful for logging, monitoring, or UI updates.
     *          Must be non-blocking and thread-safe.
     */
    std::function<void(GuardianState, GuardianState)> onStateChangeCallback;


    // ==================== Private Member Functions ====================

    /**
     * @brief Main event processing function implementing state machine logic
     * @param event The guardian event to process
     *
     * @details Implements the core state transition logic:
     *
     * **SAFE_MONITORING State:**
     * - Counts consecutive bad frames
     * - Transitions to FROZEN_UNSAFE when badCount >= freezeCount
     * - Resets badCount on good frames
     *
     * **FROZEN_UNSAFE State:**
     * - Ignores all frame events (stays frozen for safety)
     * - Only OPERATOR_ACK event can trigger transition to RESET_PENDING
     * - Prevents automatic recovery without human verification
     *
     * **RESET_PENDING State:**
     * - Counts consecutive good frames
     * - Transitions to SAFE_MONITORING when goodCount >= recoverCount
     * - Resets goodCount if bad frame detected during recovery
     *
     * @note This function is non-blocking and has deterministic execution time
     * @warning Must not be called directly - use processFrame() or operatorAcknowledge()
     */
    void processEvent(GuardianEvent event);

    /**
     * @brief Executes the specified action (freeze or clear)
     * @param action The action to execute
     *
     * @details Separated from transition logic for clean separation of concerns.
     *          Updates motionBlocked flag and invokes registered callbacks.
     *
     * **Action Execution:**
     * - FREEZE_NOW: Sets motionBlocked=true, calls onFreezeCallback
     * - CLEAR_FREEZE: Sets motionBlocked=false, calls onClearFreezeCallback
     * - NONE: No operation (state change only)
     *
     * @note Callbacks are executed synchronously in the calling thread
     */
    void executeAction(GuardianAction action);

    /**
     * @brief Handles state transition with associated action
     * @param newState The target state to transition to
     * @param action The action to execute during transition
     *
     * @details Central function ensuring all transitions follow consistent pattern:
     * 1. Store old state for logging/callbacks
     * 2. Update currentState atomically
     * 3. Execute associated action (freeze/clear)
     * 4. Notify state change callback (if registered and state changed)
     * 5. Log transition for debugging
     *
     * This design prevents inconsistent state updates and ensures all transitions
     * are properly logged and communicated to external systems.
     *
     * @note Thread-safe due to atomic state variable
     */
    void transitionTo(GuardianState newState, GuardianAction action);

    /**
     * @brief Logs state transition information
     * @param from Previous state
     * @param to New state
     *
     * @details Only logs when actual state change occurs (from != to).
     *          Includes current counter values for debugging.
     *          Output format: [TRANSITION] STATE1 -> STATE2 | bad_count=X good_count=Y
     */
    void logTransition(GuardianState from, GuardianState to);

    /**
     * @brief Logs action execution
     * @param action Description of the action being executed
     *
     * @details Provides clear audit trail of safety-critical actions.
     */
    void logAction(const std::string& action);

public:
    // ==================== Public Member Functions ====================

    /**
     * @brief Constructs a GuardianStateMachine with configurable thresholds
     * @param fc Freeze count - number of consecutive bad frames to trigger freeze (default: 30)
     * @param rc Recover count - number of consecutive good frames to clear freeze (default: 3)
     *
     * @details Initializes state machine in SAFE_MONITORING state with motion allowed.
     *          All counters start at 0, callbacks are null until registered.
     *
     * @note Adjust thresholds based on frame rate and safety requirements
     * @warning Thresholds must be > 0, no validation performed
     */
    GuardianStateMachine(int fc = 30, int rc = 3);

    /**
     * @brief Processes a frame status and updates state machine
     * @param status The status of the received frame (FRAME_GOOD or FRAME_BAD)
     *
     * @details Convenience wrapper that converts FrameStatus to GuardianEvent
     *          and calls processEvent(). Use this for frame-based monitoring.
     */
    void processFrame(FrameStatus status);

    /**
     * @brief Handles operator acknowledgment input
     *
     * @details Triggers OPERATOR_ACK event, allowing transition from FROZEN_UNSAFE
     *          to RESET_PENDING state. Required for recovery after freeze.
     *
     * **Safety Protocol:**
     * 1. System detects danger → freezes automatically
     * 2. Operator investigates and resolves issue
     * 3. Operator calls operatorAcknowledge()
     * 4. System verifies stability with consecutive good frames
     * 5. System resumes motion automatically
     *
     * @note Should be called only when human operator confirms safety
     * @note Has no effect if not in FROZEN_UNSAFE state
     * @warning Do not automate this call - requires human verification
     */
    void operatorAcknowledge();
    
    /**
     * @brief Registers callback for freeze action
     * @param callback Function to call when motion is frozen
     *
     * @details Callback should implement emergency stop or motion blocking.
     *          Called synchronously during state transition to FROZEN_UNSAFE.
     */
    void setOnFreezeCallback(std::function<void()> callback);

    /**
     * @brief Registers callback for clear-freeze action
     * @param callback Function to call when freeze is cleared
     *
     * @details Callback should resume motion control.
     *          Called synchronously during state transition from RESET_PENDING to SAFE_MONITORING.
     */
    void setOnClearFreezeCallback(std::function<void()> callback);

    /**
     * @brief Registers callback for state transitions
     * @param callback Function to call on state change (receives old and new state)
     *
     * @details Called synchronously after state update but before action execution.
     *          Useful for logging, monitoring, or UI updates.
     */
    void setOnStateChangeCallback(std::function<void(GuardianState, GuardianState)> callback);
    
    /**
     * @brief Gets the current state
     * @return Current GuardianState enum value
     *
     * @details Thread-safe read of atomic state variable.
     *          Can be called from any thread without synchronization.
     *
     * @note Returns snapshot of state at call time, may change immediately after
     */
    GuardianState getState() const;

    /**
     * @brief Checks if motion is currently blocked
     * @return true if motion is blocked (frozen), false otherwise
     *
     * @details Thread-safe read of atomic motionBlocked flag.
     *          This is the primary query for motion control systems.
     *
     * @note Returns snapshot at call time, may change immediately after
     */
    bool isMotionBlocked() const;

    /**
     * @brief Checks if motion is currently allowed
     * @return true if motion is allowed, false if blocked
     *
     * @details Inverse of isMotionBlocked() for convenience.
     *          Semantically clearer in some contexts.
     *
     * @note Returns snapshot at call time, may change immediately after
     */
    bool isMotionAllowed() const;

    /**
     * @brief Gets current count of consecutive bad frames
     * @return Number of consecutive bad frames detected
     *
     * @details Useful for monitoring how close system is to freezing.
     *          Value is only meaningful in SAFE_MONITORING state.
     *
     * @note Thread-safe atomic read
     */
    int getBadCount() const;

    /**
     * @brief Gets current count of consecutive good frames
     * @return Number of consecutive good frames during recovery
     *
     * @details Useful for monitoring recovery progress.
     *          Value is only meaningful in RESET_PENDING state.
     *
     * @note Thread-safe atomic read
     */
    int getGoodCount() const;
    
    /**
     * @brief Converts state enum to human-readable string
     * @param state The state to convert
     * @return String representation of the state
     *
     * @details Returns one of:
     * - "SAFE_MONITORING"
     * - "FROZEN_UNSAFE"
     * - "RESET_PENDING"
     * - "UNKNOWN" (defensive fallback for invalid state)
     *
     * @note Useful for logging and UI display
     */
    std::string stateToString(GuardianState state) const;

    /**
     * @brief Gets string representation of current state
     * @return String representation of currentState
     *
     * @details Convenience wrapper around stateToString(currentState).
     *          Thread-safe due to atomic state variable.
     */
    std::string getCurrentStateString() const;

    /**
     * @brief Prints comprehensive status information to console
     *
     * @details Outputs current state, motion status, and counter values.
     *          Useful for debugging and system monitoring.
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
     * @note Thread-safe - all reads are atomic
     */
    void printStatus() const;
};

#endif // GUARDIAN_STATE_MACHINE_HPP