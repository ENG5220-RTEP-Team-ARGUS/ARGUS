#pragma once

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
    };

    struct MotionSmokeTestOptions {
        SmokeJoint joint = SmokeJoint::All;
    };

    AppController() noexcept;
    ~AppController() noexcept;

    int runGuardianScenarioDemo();
    int runButtonTest();
    int runMotionHomePose();
    int runMotionSmokeTest(const MotionSmokeTestOptions& options);
    int runFullPipelineDemo(const LiveTestOptions& options);
    int runLiveMarkerTest(const LiveTestOptions& options);

private:
    MotionController motion_controller_;
};
