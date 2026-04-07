#pragma once

#include "MotionController.hpp"

class AppController {
public:
    struct LiveTestOptions {
        int camera_index = 0;
        int expected_marker_id = 23;
        bool auto_ack = false;
    };

    AppController() noexcept;
    ~AppController() noexcept;

    int runGuardianScenarioDemo();
    int runMotionSmokeTest();
    int runLiveMarkerTest(const LiveTestOptions& options);

private:
    MotionController motion_controller_;
};
