/**
 * @file AppController.hpp
 * @brief Top-level runtime mode controller for ARGUS.
 *
 * AppController owns the high-level run modes used by the CLI entry point.
 * Each run method executes one mode (live test, smoke test, button test, etc.)
 * and returns a process-style status code.
 */

#pragma once

#include "CameraCapture.hpp"
#include "MotionController.hpp"

/**
 * @brief Orchestrates runtime modes for ARGUS.
 */
class AppController {
public:
    /**
     * @brief Joint selector used by motion smoke-test mode.
     */
    enum class SmokeJoint {
        All,    ///< Run the full all-joint smoke sequence.
        Base,   ///< Run smoke test for base joint only.
        Lower,  ///< Run smoke test for lower joint only.
        Upper,  ///< Run smoke test for upper joint only.
        Grip,   ///< Run smoke test for gripper joint only.
    };

    /**
     * @brief Options used by live test and camera backend check modes.
     */
    struct LiveTestOptions {
        int camera_index = 0;  ///< Camera index passed to CameraCapture.
        int expected_marker_id = 23;  ///< Expected marker ID (legacy CLI option).
        bool auto_ack = false;  ///< Auto-send acknowledge after freeze for testing.
        CameraCapture::BackendPreference backend_preference =
            CameraCapture::BackendPreference::Auto;  ///< Requested camera backend policy.
    };

    /**
     * @brief Options for the motion smoke-test mode.
     */
    struct MotionSmokeTestOptions {
        SmokeJoint joint = SmokeJoint::All;  ///< Joint subset to smoke-test.
    };

    /**
     * @brief Construct the application controller.
     */
    AppController() noexcept;

    /**
     * @brief Destroy the application controller.
     */
    ~AppController() noexcept;

    /**
     * @brief Run the scripted guardian scenario demonstration.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runGuardianScenarioDemo();

    /**
     * @brief Run physical-button diagnostics.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runButtonTest();

    /**
     * @brief Run interactive raw-pulse servo calibration mode.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runServoCalibration();

    /**
     * @brief Run interactive servo console mode.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runInteractiveServoConsole();

    /**
     * @brief Move the arm to the configured home pose and exit.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runMotionHomePose();

    /**
     * @brief Run motion-only smoke tests using the selected joint scope.
     * @param options Smoke-test options.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runMotionSmokeTest(const MotionSmokeTestOptions& options);

    /**
     * @brief Run camera backend validation/check mode.
     * @param options Camera/backend options.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runCameraBackendCheck(const LiveTestOptions& options);

    /**
     * @brief Run live safety supervision mode.
     * @param options Live mode options.
     * @return Exit code (`0` success, non-zero failure).
     */
    int runLiveMarkerTest(const LiveTestOptions& options);

private:
    MotionController motion_controller_;
};
