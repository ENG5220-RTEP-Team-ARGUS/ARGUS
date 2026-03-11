#ifndef GUARDIAN_STATE_MACHINE_HPP  // To prevent multiple inclusion of this header file
#define GUARDIAN_STATE_MACHINE_HPP  // To define the guard macro (included only once)

#include <string>
#include <functional>


// Enum Definitions
enum class GuardianState {
    SAFE_MONITORING,   // Normal monitoring state (system running normally)
    FROZEN_UNSAFE,     // Unsafe detected -> motion is frozen
    RESET_PENDING      // Operator acknowledged -> waiting for stable recovery
};

enum class FrameStatus {
    FRAME_GOOD,        // Frame is in safe condition
    FRAME_BAD          // Frame indicates unsafe condition
};

enum class GuardianEvent {
    FRAME_GOOD,        // Event triggered when a safe frame is received
    FRAME_BAD,         // Event triggered when an unsafe frame is received
    OPERATOR_ACK       // Event triggered when operator acknowledges/reset
};

enum class GuardianAction {
    NONE,              // No action required
    FREEZE_NOW,        // Immediately freeze/block motion
    CLEAR_FREEZE       // Clear freeze and resume motion
};


// Class Declaration
class GuardianStateMachine {
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
    GuardianStateMachine(int fc = 2, int rc = 3);    // Note: To be adjusted accordingly in future revisions
    
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