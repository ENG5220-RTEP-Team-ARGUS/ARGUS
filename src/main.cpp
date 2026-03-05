/*
    GuardianStateMachine.hpp access to:
    - GuardianStateMachine class
    - FrameStatus enum (FRAME_GOOD / FRAME_BAD)
    - GuardianState enum (SAFE_MONITORING / FROZEN_UNSAFE / RESET_PENDING)
    - Callback setter methods
*/
#include "GuardianStateMachine.hpp"
#include <iostream>

/*
    Callback: robotArmFreezeHandler()

    Purpose:
    - This simulates the robot arm would do when the guardian decides the environment is unsafe
    - To execute motion such as:
        - Emergency stop to motor drivers
        - Disable PWM outputs
        - Cut power relay
        - Stop robot motion planning

    Returns:
    - Only print a message to show the callback was triggered
*/
void robotArmFreezeHandler() {
    std::cout << ">>> ROBOTIC ARM: Emergency stop activated! <<<" << std::endl;
}


/*
    Callback: robotArmClearFreezeHandler()

    Purpose:
    - Simulates resuming robotic motion when the guardian decides it is safe again
    - To execute motion such as:
        - Re-enable motor control
        - Reset and allow motion commands again
*/
void robotArmClearFreezeHandler() {
    std::cout << ">>> ROBOTIC ARM: Motion resumed, system operational <<<" << std::endl;
}


/*
    Callback: stateChangeHandler(from, to)

    Purpose:
    - Triggered whenever the state machine changes state

    Parameters:
    - from: previous state
    - to: new state
*/
void stateChangeHandler(GuardianState from, GuardianState to) {
    std::cout << ">>> STATE CHANGE NOTIFICATION: System transitioned <<<" << std::endl;
}


/*
    main()

    Purpose:
    - Entry point of the program
    - Creates the GuardianStateMachine instance
    - Registers callbacks
    - Runs a sequence of test scenarios to validate logic
*/
int main() {
    /*
        Create a GuardianStateMachine object
        Parameters:
        - freezeCount = 2: freeze after 2 consecutive bad frames
        - recoverCount = 3: clear freeze after 3 consecutive good frames

        Safety meaning:
        - freezeCount avoids freezing due to one noisy/false-positive bad frame
        - recoverCount avoids resuming motion too early after operator acknowledgment, requiring stability before allowing motion again.
    */
    GuardianStateMachine guardian(2, 3);

    // Register callback functions
    guardian.setOnFreezeCallback(robotArmFreezeHandler);           // called when FREEZE_NOW happens
    guardian.setOnClearFreezeCallback(robotArmClearFreezeHandler); // called when CLEAR_FREEZE happens
    guardian.setOnStateChangeCallback(stateChangeHandler);         // called on state transitions

    // Print a banner to show the beginning of tests
    std::cout << "\n========== GUARDIAN STATE MACHINE TEST ==========\n" << std::endl;

    /*
        Scenario 1: Normal Operation

        Expected:
        - Two good frames should keep the system in SAFE_MONITORING
        - badCount remains 0
        - motion remains allowed
    */
    std::cout << "Scenario 1: Normal Operation" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Feed a good frame
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Another good frame
    guardian.printStatus();                           // Print state + counters

    /*
        Scenario 2: Single Bad Frame

        Expected:
        - A single bad frame increments badCount to 1
        - Next good frame resets badCount to 0
        - Should NOT freeze because freezeCount = 2
    */
    std::cout << "Scenario 2: Single Bad Frame" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_BAD);  // badCount = 1 (below threshold freezeCount=2)
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // resets badCount back to 0
    guardian.printStatus();

    /*
        Scenario 3: freezeCount Consecutive Bad Frames (Freeze)

        Expected:
        - Two consecutive bad frames → badCount reaches freezeCount=2
        - Guardian transitions SAFE_MONITORING -> FROZEN_UNSAFE
        - FREEZE_NOW action triggers:
            - motionBlocked = true
            - robotArmFreezeHandler() called
    */
    std::cout << "Scenario 3: freezeCount Consecutive Bad Frames (Freeze)" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_BAD);  // badCount = 1
    guardian.processFrame(FrameStatus::FRAME_BAD);  // badCount = 2 -> freeze triggered
    guardian.printStatus();

    /*
        Scenario 4: Frames During Frozen State

        Expected:
        - Even if frames become GOOD, the system should NOT automatically unfreeze, it stays frozen until operator acknowledgment
        - This is a safety-critical design principle as automatic restart is dangerous
    */
    std::cout << "Scenario 4: Frames During Frozen State" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Still frozen
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // Still frozen
    guardian.printStatus();

    /*
        Scenario 5: Operator Acknowledgment

        Expected:
        - OPERATOR_RESET moves state from FROZEN_UNSAFE -> RESET_PENDING
        - Still frozen (motionBlocked remains true)
        - Now the guardian begins counting good frames
    */
    std::cout << "Scenario 5: Operator Acknowledgment/Reset" << std::endl;
    guardian.operatorAcknowledge();            // Human acknowledges unsafe condition
    guardian.printStatus();

    /*
        Scenario 6: Bad Frame During Reset

        Expected:
        - In RESET_PENDING, guardian counts consecutive good frames
        - If a bad frame occurs, goodCount resets to 0
        - This prevents “false recovery” when environment becomes unstable again
    */
    std::cout << "Scenario 6: Bad Frame During Reset" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 1
    guardian.processFrame(FrameStatus::FRAME_BAD);  // goodCount reset to 0
    guardian.printStatus();

    /*
        Scenario 7: recoverCount Consecutive Good Frames (Clear Freeze)

        Expected:
        - Need 3 consecutive good frames (recoverCount=3) to recover
        - After third good frame:
            - state RESET_PENDING -> SAFE_MONITORING
            - CLEAR_FREEZE action triggers:
                - motionBlocked = false
                - robotArmClearFreezeHandler() called
            - counters reset to 0
    */
    std::cout << "Scenario 7: recoverCount Consecutive Good Frames (Clear Freeze)" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 1
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 2
    guardian.processFrame(FrameStatus::FRAME_GOOD);   // goodCount = 3 -> clear freeze and resume
    guardian.printStatus();

    std::cout << "========== TEST COMPLETE ==========\n" << std::endl;  // Print messages to show tests have completed
    return 0;
}