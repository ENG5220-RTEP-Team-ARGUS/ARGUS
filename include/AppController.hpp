#pragma once

#include "CameraCapture.hpp"
#include "MotionController.hpp"

class AppController {
public:
    enum class SmokeJoint {
        All,
        Base,
        Lower,
        Upper,
        Grip,
    };

    struct LiveTestOptions {
        int camera_index = 0;
        int expected_marker_id = 23;
        bool auto_ack = false;
        CameraCapture::BackendPreference backend_preference =
            CameraCapture::BackendPreference::Auto;
    };

    struct MotionSmokeTestOptions {
        SmokeJoint joint = SmokeJoint::All;
    };

    AppController() noexcept;
    ~AppController() noexcept;

    int runGuardianScenarioDemo();
    int runButtonTest();
    int runServoCalibration();
    int runInteractiveServoConsole();
    int runMotionHomePose();
    int runMotionSmokeTest(const MotionSmokeTestOptions& options);
    int runFullPipelineDemo(const LiveTestOptions& options);
    int runLiveMarkerTest(const LiveTestOptions& options);

private:
    MotionController motion_controller_;
};
