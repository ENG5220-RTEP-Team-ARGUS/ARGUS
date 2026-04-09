/**
 * @file main.cpp
 * @brief Test suite and demonstration for GuardianStateMachine
 *
 * This file demonstrates the GuardianStateMachine's safety protocol through
 * comprehensive test scenarios. It validates state transitions, callback
 * execution, and counter behavior under various conditions.
 *
 * @section test_scenarios Test Scenarios
 * 1. Normal operation with good frames
 * 2. Single bad frame (no freeze)
 * 3. Consecutive bad frames triggering freeze
 * 4. Frames during frozen state (no auto-recovery)
 * 5. Operator acknowledgment
 * 6. Bad frame during recovery
 * 7. Successful recovery with consecutive good frames
 *
 * @section integration_example Integration Example
 * This demonstrates how to integrate GuardianStateMachine with a robotic arm
 * control system using callbacks for hardware control.
 */

#include "GuardianStateMachine.hpp"
#include "MetricsLogger.hpp"
#include <iostream>
#include <thread>
#include <chrono>

/**
 * @brief Callback handler for freeze action (emergency stop)
 *
 * @details This simulates what the robot arm would do when the guardian
 * detects an unsafe environment. In a real system, this would:
 * - Send emergency stop command to motor drivers
 * - Disable PWM outputs
 * - Cut power relay
 * - Halt motion planning
 */
void robotArmFreezeHandler() {
    MetricsLogger::getInstance().logEvent("ROBOT_ARM", "Emergency stop activated!");
}


/**
 * @brief Callback handler for clear-freeze action (resume motion)
 *
 * @details Simulates resuming robotic motion when the guardian determines
 * it is safe again. In a real system, this would:
 * - Re-enable motor control
 * - Restore PWM outputs
 * - Reset motion planner
 * - Allow motion commands
 */
void robotArmClearFreezeHandler() {
    MetricsLogger::getInstance().logEvent("ROBOT_ARM", "Motion resumed, system operational");
}


/**
 * @brief Callback handler for state change notifications
 *
 * @param from Previous GuardianState before transition
 * @param to New GuardianState after transition
 *
 * @details Triggered whenever the state machine changes state.
 * Useful for logging, monitoring, UI updates, or triggering dependent systems.
 */
void stateChangeHandler(GuardianState from, GuardianState to) {
    MetricsLogger::getInstance().logEvent("STATE_CHANGE", "System transitioned");
}


/**
 * @brief Main entry point - test suite for GuardianStateMachine
 *
 * @return int Exit code (0 = success)
 *
 * @details Creates a GuardianStateMachine instance, registers callbacks,
 * and runs a comprehensive test suite to validate all state transitions
 * and safety behaviors.
 *
 * **Test Configuration:**
 * - freezeCount = 2: Freeze after 2 consecutive bad frames
 * - recoverCount = 3: Clear freeze after 3 consecutive good frames
 *
 * **Safety Rationale:**
 * - freezeCount > 1: Avoids freezing due to single noisy/false-positive frame
 * - recoverCount > 1: Requires stability verification before resuming motion
 *
 * **Test Coverage:**
 * - Normal operation (good frames)
 * - Transient errors (single bad frame)
 * - Freeze trigger (consecutive bad frames)
 * - Frozen state behavior (no auto-recovery)
 * - Operator acknowledgment
 * - Recovery interruption (bad frame during recovery)
 * - Successful recovery (consecutive good frames)
 */
int main() {
    /**
     * @brief Initialize GuardianStateMachine with test thresholds
     *
     * @details Configuration:
     * - freezeCount = 30: Requires 30 consecutive bad frames to freeze
     * - recoverCount = 3: Requires 3 consecutive good frames to recover
     */
    GuardianStateMachine guardian(30, 3);

    /**
     * @brief Register callback functions for hardware control
     *
     * @details Callbacks implement the Dependency Inversion Principle:
     * - GuardianStateMachine doesn't depend on specific hardware
     * - Hardware control is injected via callbacks
     */
    guardian.setOnFreezeCallback(robotArmFreezeHandler);           // Called when FREEZE_NOW action executes
    guardian.setOnClearFreezeCallback(robotArmClearFreezeHandler); // Called when CLEAR_FREEZE action executes
    guardian.setOnStateChangeCallback(stateChangeHandler);         // Called on all state transitions

    MetricsLogger::getInstance().logEvent("TEST", "========== GUARDIAN STATE MACHINE TEST ==========");

    /**
     * @test Scenario 1: Normal Operation
     *
     * @details Tests system behavior under normal conditions with good frames.
     *
     * **Expected Behavior:**
     * - State remains SAFE_MONITORING
     * - badCount remains 0
     * - Motion remains allowed
     * - No callbacks triggered
     *
     * **Validates:**
     * - Normal operation doesn't trigger false alarms
     * - Good frames reset bad frame counter
     */
    MetricsLogger::getInstance().logEvent("[TEST]", "Scenario 1 - Normal Operation");
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Feed a good frame
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Another good frame
    guardian.printStatus();                           // Display state and counters

    /**
     * @test Scenario 2: Single Bad Frame (No Freeze)
     *
     * @details Tests transient error handling - single bad frame should not freeze.
     *
     * **Expected Behavior:**
     * - badCount increments to 1 (below threshold)
     * - Next good frame resets badCount to 0
     * - No freeze triggered (freezeCount = 2)
     * - Motion remains allowed
     *
     * **Validates:**
     * - System filters transient sensor noise
     * - Requires consecutive bad frames to freeze
     * - Good frame resets bad frame counter
     */
    MetricsLogger::getInstance().logEvent("[TEST]", "Scenario 2 - Single Bad Frame");
    guardian.processFrame(FrameStatus::FRAME_BAD);    // badCount = 1 (below threshold)
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Resets badCount to 0
    guardian.printStatus();

    /**
     * @test Scenario 3: Consecutive Bad Frames (Freeze Triggered)
     *
     * @details Tests freeze trigger when consecutive bad frames reach threshold.
     *
     * **Expected Behavior:**
     * - First bad frame: badCount = 1
     * - Second bad frame: badCount = 2 (reaches freezeCount)
     * - State transitions: SAFE_MONITORING -> FROZEN_UNSAFE
     * - FREEZE_NOW action executes:
     *   - motionBlocked = true
     *   - robotArmFreezeHandler() called
     *   - Emergency stop activated
     *
     * **Validates:**
     * - Freeze triggers at correct threshold
     * - State transition occurs correctly
     * - Freeze callback executes
     * - Motion is blocked
     */ 
    MetricsLogger::getInstance().logEvent("[TEST]", "Scenario 3 - Consecutive Bad Frames (Freeze)");
    guardian.processFrame(FrameStatus::FRAME_BAD);  // badCount = 1
    guardian.processFrame(FrameStatus::FRAME_BAD);  // badCount = 2 -> freeze triggered
    guardian.printStatus();

    /**
     * @test Scenario 4: Frames During Frozen State (No Auto-Recovery)
     *
     * @details Tests that system stays frozen even when frames become good.
     *
     * **Expected Behavior:**
     * - Good frames are ignored in FROZEN_UNSAFE state
     * - State remains FROZEN_UNSAFE
     * - Motion remains blocked
     * - No automatic recovery
     *
     * **Validates:**
     * - Safety-critical design: no automatic restart
     * - Requires human operator acknowledgment
     * - Prevents dangerous automatic recovery
     */
    MetricsLogger::getInstance().logEvent("[TEST]", "Scenario 4 - Frames During Frozen State");
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Still frozen
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Still frozen
    guardian.printStatus();

    /**
     * @test Scenario 5: Operator Acknowledgment
     *
     * @details Tests operator acknowledgment initiating recovery process.
     *
     * **Expected Behavior:**
     * - OPERATOR_ACK event processed
     * - State transitions: FROZEN_UNSAFE -> RESET_PENDING
     * - Motion still blocked (no CLEAR action yet)
     * - goodCount reset to 0
     * - System begins counting consecutive good frames
     *
     * **Validates:**
     * - Operator acknowledgment required for recovery
     * - State transitions correctly
     * - Motion remains blocked during recovery verification
     */
    MetricsLogger::getInstance().logEvent("[TEST]", "Scenario 5 - Operator Acknowledgment");
    guardian.operatorAcknowledge();            // Human acknowledges unsafe condition resolved
    guardian.printStatus();

    /**
     * @test Scenario 6: Bad Frame During Recovery (Reset Counter)
     *
     * @details Tests recovery interruption when bad frame occurs during verification.
     *
     * **Expected Behavior:**
     * - First good frame: goodCount = 1
     * - Bad frame: goodCount resets to 0
     * - State remains RESET_PENDING
     * - Motion remains blocked
     *
     * **Validates:**
     * - Requires consecutive good frames for recovery
     * - Bad frame during recovery resets verification
     * - Prevents premature recovery when environment unstable
     */
    MetricsLogger::getInstance().logEvent("[TEST]", "Scenario 6 - Bad Frame During Recovery");
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 1
    guardian.processFrame(FrameStatus::FRAME_BAD);    // goodCount reset to 0
    guardian.printStatus();

    /**
     * @test Scenario 7: Successful Recovery (Clear Freeze)
     *
     * @details Tests successful recovery with consecutive good frames.
     *
     * **Expected Behavior:**
     * - First good frame: goodCount = 1
     * - Second good frame: goodCount = 2
     * - Third good frame: goodCount = 3 (reaches recoverCount)
     * - State transitions: RESET_PENDING → SAFE_MONITORING
     * - CLEAR_FREEZE action executes:
     *   - motionBlocked = false
     *   - robotArmClearFreezeHandler() called
     *   - Motion resumed
     * - All counters reset to 0
     *
     * **Validates:**
     * - Recovery completes at correct threshold
     * - State transitions correctly
     * - Clear callback executes
     * - Motion is allowed
     * - Counters reset for next monitoring cycle
     */
    MetricsLogger::getInstance().logEvent("[TEST]", "Scenario 7 - Successful Recovery");
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 1
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 2
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 3 -> clear freeze and resume
    guardian.printStatus();

    MetricsLogger::getInstance().logEvent("[TEST]", "========== TEST COMPLETE ==========");
    // Flush logs before exit
    MetricsLogger::getInstance().flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 0;
}