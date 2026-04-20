/**
 * @file GuardianStateMachine.cpp
 * @brief Implementation of the guardian safety state machine.
 */

#include "GuardianStateMachine.hpp"
#include <iostream>

/*
    GuardianStateMachine()

    Note: enums (GuardianState, GuardianEvent, GuardianAction, FrameStatus)

    Purpose:
    - Set the initial state and internal counters
    - Store the thresholds freezeCount and recoverCount (bad frames to freeze, good frames to recover)
    - Ensure motion starts as allowed (not blocked)
    - Initialize callbacks to null so we can safely check before calling

    Parameters:
    - fc (freezeCount): number of consecutive bad frames required to trigger a freeze
    - rc (recoverCount): number of consecutive good frames required to clear a freeze
*/
GuardianStateMachine::GuardianStateMachine(int fc, int rc) 
    : currentState(GuardianState::SAFE_MONITORING),  // Start in normal monitoring mode
      badCount(0),                                   // No bad frames detected yet
      goodCount(0),                                  // No good frames counted for recovery yet
      freezeCount(fc),                               // Store freeze trigger threshold
      recoverCount(rc),                              // Store recovery threshold
      motionBlocked(false),                          // Motion should be allowed at beginning/start-up
      onFreezeCallback(nullptr),                     // No freeze handler registered yet
      onClearFreezeCallback(nullptr),                // No clear handler registered yet
      onStateChangeCallback(nullptr) {               // No state change handler registered yet

    // Log initialization to verify freezeCount and recoverCount values
    std::cout << "[INIT] Guardian State Machine initialized with freezeCount=" << freezeCount << ", recoverCount=" << recoverCount << std::endl;
}


/*
    executeAction()

    Purpose:
    - Perform the actual side effects of a GuardianAction
    - This is separated from transition logic to keep state changes clean and consistent

    Design idea:
    - transitionTo() decides WHEN to execute actions
    - executeAction() defines WHAT each action does

    Actions:
    - FREEZE_NOW: block motion + call external freeze callback (if registered)
    - CLEAR_FREEZE: allow motion + call external clear callback (if registered)
    - NONE: do nothing
*/
void GuardianStateMachine::executeAction(GuardianAction action) {
    // switch decides which action to run
    switch (action) {

        case GuardianAction::FREEZE_NOW:
            // Immediately block motion (safety lock)
            motionBlocked = true;

            // Call it if external system registered a freeze handler such as emergency stop
            if (onFreezeCallback) {
                onFreezeCallback();
            }

            // Log the action for debugging/traceability
            logAction("FREEZE_NOW - Motion blocked");
            break;

        case GuardianAction::CLEAR_FREEZE:
            // Allow motion again where the guardian releases safety lock
            motionBlocked = false;

            // Call it if external system registered a clear handler to resume motion control again
            if (onClearFreezeCallback) {
                onClearFreezeCallback();
            }

            // Log the action for debugging/traceability
            logAction("CLEAR_FREEZE - Motion allowed");
            break;

        case GuardianAction::NONE:
            // No action but state change may still happen in some cases
            break;
    }
}


/*
    transitionTo()

    Purpose:
    - Central function for safe and consistent state transitions
    - Updates currentState
    - Executes action - freeze/clear
    - Notifies observers via callback
    - Logs the transition

    Explanations:
    - Prevents duplicated state-change code in multiple places
    - Guarantees transitions always:
      (1) update state
      (2) perform action
      (3) notify callback
      (4) log
*/
void GuardianStateMachine::transitionTo(GuardianState newState, GuardianAction action) {
    // Store old state to log and notify what changed
    GuardianState oldState = currentState;

    // Update to the new state
    currentState = newState;

    // Perform the associated action (freeze or clear)
    executeAction(action);

    // Notify state change only if callback is set 'AND' and state actually changed
    if (onStateChangeCallback && oldState != newState) {
        onStateChangeCallback(oldState, newState);
    }

    // Log the action for debugging/traceability
    logTransition(oldState, newState);
}


/*
    logTransition()

    Purpose:
    - Print transition info only when an actual state change occurs
    - Also prints current counters for debugging and traceability
*/
void GuardianStateMachine::logTransition(GuardianState from, GuardianState to) {
    // Only print if transition happened
    if (from != to) {
        std::cout << "[TRANSITION] " << stateToString(from)
                  << " -> " << stateToString(to)
                  << " | bad_count=" << badCount
                  << " good_count=" << goodCount << std::endl;
    }
}


/*
    logAction()

    Purpose:
    - Log actions as separate messages from transitions
    - This separation makes logs clearer:
      transitions = state changes
      actions = physical/system effects (freeze/clear)
*/
void GuardianStateMachine::logAction(const std::string& action) {
    std::cout << "[ACTION] " << action << std::endl;
}


/*
    processEvent()

    Purpose:
    - Main section of the state machine
    - Takes a GuardianEvent (FRAME_GOOD / FRAME_BAD / OPERATOR_ACK) and updates counters + changes state when rules are met

    Key safety behavior:
    1) SAFE_MONITORING:
       - count consecutive bad frames
       - freeze if badCount >= freezeCount
       - reset badCount on a good frame

    2) FROZEN_UNSAFE:
       - stay frozen regardless of frames
       - only OPERATOR_ACK allows move to RESET_PENDING

    3) RESET_PENDING:
       - count consecutive good frames
       - unfreeze if goodCount >= recoverCount
       - reset goodCount when a bad frame appears
*/
void GuardianStateMachine::processEvent(GuardianEvent event) {
    // Behavior depends entirely on currentState
    switch (currentState) {
        // STATE: SAFE_MONITORING
        case GuardianState::SAFE_MONITORING:

            // Assume a bad frame is detected
            if (event == GuardianEvent::FRAME_BAD) {
                // Increase consecutive bad counter
                badCount++;

                // If the threshold freezeCount is reached to max, freeze immediately
                if (badCount >= freezeCount) {
                    transitionTo(GuardianState::FROZEN_UNSAFE, GuardianAction::FREEZE_NOW);
                }

            // Assume a good frame is detected
            } else if (event == GuardianEvent::FRAME_GOOD) {
                // Reset badCount because we require consecutive bad frames
                badCount = 0;
            }
            break;

        // STATE: FROZEN_UNSAFE
        case GuardianState::FROZEN_UNSAFE:
            // Only operator reset to proceed
            if (event == GuardianEvent::OPERATOR_ACK) {
                // Start recovery checking when operator acknowledges
                goodCount = 0;

                // Move to RESET_PENDING (still frozen: no motion action yet)
                transitionTo(GuardianState::RESET_PENDING, GuardianAction::NONE);
            }
            break;

        // STATE: RESET_PENDING (RECOVERY STATE)
        case GuardianState::RESET_PENDING:
            if (event == GuardianEvent::FRAME_GOOD) {
                // Increase consecutive good counter
                goodCount++;

                // Unfreeze when consecutive good frames detected
                if (goodCount >= recoverCount) {
                    // Reset counters for clean future monitoring
                    goodCount = 0;
                    badCount = 0;

                    // Transition back to SAFE_MONITORING and clear freeze
                    transitionTo(GuardianState::SAFE_MONITORING, GuardianAction::CLEAR_FREEZE);
                }

            } else if (event == GuardianEvent::FRAME_BAD) {
                // Bad frame during recovery breaks confidence so reset goodCount and keep waiting
                goodCount = 0;
            }
            break;
    }
}


/*
    processFrame()

    Purpose:
    - Convenience wrapper so external code can pass FrameStatus rather than GuardianEvent

    Mapping:
    - FrameStatus::FRAME_GOOD  -> GuardianEvent::FRAME_GOOD
    - FrameStatus::FRAME_BAD -> GuardianEvent::FRAME_BAD
*/
void GuardianStateMachine::processFrame(FrameStatus status) {
    processEvent(status == FrameStatus::FRAME_GOOD ? GuardianEvent::FRAME_GOOD : GuardianEvent::FRAME_BAD);
}


/*
    operatorAcknowledge()

    Purpose:
    - Public API for human/operator acknowledgment
    - Logs the acknowledgment and triggers OPERATOR_ACK event

    Safety workflow:
    - Something went wrong in current situation -> freeze
    - Human confirms situation -> operator acknowledge
    - System proves stable -> resume
*/
void GuardianStateMachine::operatorAcknowledge() {
    std::cout << "[OPERATOR] Acknowledgment received" << std::endl;
    processEvent(GuardianEvent::OPERATOR_ACK);
}


/*
    Callback setters

    Purpose:
    - Allow the state machine to control external systems without hardcoding hardware-specific logic into this class to make GuardianStateMachine reusable
*/
void GuardianStateMachine::setOnFreezeCallback(std::function<void()> callback) {
    onFreezeCallback = callback; // Store freeze callback
}

void GuardianStateMachine::setOnClearFreezeCallback(std::function<void()> callback) {
    onClearFreezeCallback = callback; // Store clear-freeze callback
}

void GuardianStateMachine::setOnStateChangeCallback(std::function<void(GuardianState, GuardianState)> callback) {
    onStateChangeCallback = callback; // Store transition callback
}

/*
    getState()

    Purpose:
    - Provide safe read-only access to internal data
*/
GuardianState GuardianStateMachine::getState() const {
    return currentState; // Return the current state enum
}

bool GuardianStateMachine::isMotionBlocked() const {
    return motionBlocked; // True = frozen
}

bool GuardianStateMachine::isMotionAllowed() const {
    return !motionBlocked; // False = Resume
}

int GuardianStateMachine::getBadCount() const {
    return badCount; // Return current consecutive bad frame count
}

int GuardianStateMachine::getGoodCount() const {
    return goodCount; // Return current consecutive good frame count
}


/*
    stateToString()

    Purpose:
    - Convert enum to readable text for logs and UI debugging
*/
std::string GuardianStateMachine::stateToString(GuardianState state) const {
    switch (state) {
        case GuardianState::SAFE_MONITORING: return "SAFE_MONITORING";
        case GuardianState::FROZEN_UNSAFE:   return "FROZEN_UNSAFE";
        case GuardianState::RESET_PENDING:   return "RESET_PENDING";
        default:                             return "UNKNOWN"; // Defensive fallback
    }
}


/*
    getCurrentStateString()
    - To get string form of current state
*/
std::string GuardianStateMachine::getCurrentStateString() const {
    return stateToString(currentState);
}


/*
    printStatus()

    Purpose:
    - Print a full snapshot of guardian internal status for testing and debugging
*/
void GuardianStateMachine::printStatus() const {
    std::cout << "\n=== GUARDIAN STATUS ===" << std::endl;
    std::cout << "State: " << getCurrentStateString() << std::endl;
    std::cout << "Motion: " << (motionBlocked ? "BLOCKED" : "ALLOWED") << std::endl;
    std::cout << "Bad Count: " << badCount << "/" << freezeCount << std::endl;
    std::cout << "Good Count: " << goodCount << "/" << recoverCount << std::endl;
    std::cout << "=======================\n" << std::endl;
}
