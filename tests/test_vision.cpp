#include "Types.hpp"
#include "VisionConfig.hpp"
#include "VisionProcessor.hpp"

#include <cassert>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

int main() {
    VisionConfig config;
    config.safeZoneXMin = 0;
    config.safeZoneYMin = 0;
    config.safeZoneXMax = 100;
    config.safeZoneYMax = 100;
    config.depthCheckEnabled = true;
    config.depthHueLower1 = 50;
    config.depthHueUpper1 = 70;
    config.depthHueLower2 = 255;
    config.depthHueUpper2 = 0;
    config.depthSatMin = 200;
    config.depthSatMax = 255;
    config.depthValMin = 200;
    config.depthValMax = 255;
    config.depthPixelThreshold = 50;

    VisionProcessor vision(config);

    const auto now = std::chrono::steady_clock::now();

    cv::Mat safe_frame(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    const SafetyResult safe_result = vision.process(safe_frame, now);
    assert(safe_result.state == SafetyState::SAFE);

    cv::Mat unsafe_frame = safe_frame.clone();
    cv::rectangle(unsafe_frame,
                  cv::Rect(20, 20, 20, 20),
                  cv::Scalar(0, 255, 0),
                  cv::FILLED);
    const SafetyResult unsafe_result =
        vision.process(unsafe_frame, now + std::chrono::milliseconds(33));
    assert(unsafe_result.state == SafetyState::DEPTH_EXCEEDED);
    assert(unsafe_result.processing_time.count() >= 0);

    return 0;
}
