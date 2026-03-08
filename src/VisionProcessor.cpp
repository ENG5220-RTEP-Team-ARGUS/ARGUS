/*
 VisionProcessor.cpp
 Implementation of the ARGUS vision safety evaluator.
 
 This file contains the logic for ArUco marker detection and
 safety evaluation.
*/

#include "VisionProcessor.hpp"
#include <cmath>
// Constructor
// All expensive one-time setup (dictionary, detector) is done in the
// constructor so process() stays fast and its latency stays predictable.

VisionProcessor::VisionProcessor(const VisionConfig& config)
    : config_(config),
      // DICT_6X6_250: small dictionary = higher inter-marker distance =
      // fewer false positives. Important in a safety-critical context.
      // Built once here — not per frame — to keep process() latency bounded.
      // dictionary_(cv::aruco::getPredefinedDictionary(config.dictionaryId)),
      
      // TODO: verify OpenCV version on Raspberry Pi before deployment.
      // Current code uses OpenCV 4.7+ API (Dictionary by value, not cv::Ptr).
      // If Pi has OpenCV < 4.7, revert to: cv::Ptr<cv::aruco::Dictionary> dictionary_;
      // Check with: pkg-config --modversion opencv4
      dictionary_(cv::aruco::getPredefinedDictionary(
                        static_cast<cv::aruco::PredefinedDictionaryType>(config.dictionaryId))),
      // Construct the detector with default parameters and the dictionary
      // above. DetectorParameters uses its defaults unless overridden.
      // Constructed here rather than per-frame to avoid repeated heap
      // allocation inside process(), keeping latency bounded.
      detector_(dictionary_, detectorParams_)
{}

//  process()
//  Six-stage pipeline: detect -> validate ID-> centroid->
//  zone check-> speed check-> orientation check.
SafetyResult VisionProcessor::process(
    const cv::Mat& frame,
    std::chrono::steady_clock::time_point captureTimestamp)
{
    // Record entry time so processing_time can be measured at each return point.
    const auto startTime = std::chrono::steady_clock::now();

    // Helper lambda: builds a SafetyResult with correct timestamps and
    // processing_time without duplicating this logic at every return point.
    auto makeResult = [&](SafetyState state) -> SafetyResult {
        const auto now = std::chrono::steady_clock::now();
        return SafetyResult{
            state,
            now,
            std::chrono::duration_cast<std::chrono::microseconds>(now - startTime)
        };
    };

    // Stage 1: Detect all markers in frame

    std::vector<int> markerIds; // An empty list of integers that OpenCV will fill with the ID of each detected marker.
    std::vector<std::vector<cv::Point2f>> markerCorners; // A list of lists. Each detected marker gets its own inner list of four Point2f corners (top-left, top-right, bottom-right, bottom-left)
    std::vector<std::vector<cv::Point2f>> rejected;  // unused, required by API

    // OpenCV analyses the frame - thresholds it, finds square contours, 
    // reads the binary pattern inside each square - and populates 
    // markerCorners and markerIds with whatever it finds.
    detector_.detectMarkers(frame, markerCorners, markerIds, rejected);

    // Stage 2: Early exit if nothing detected 

    // Quick empty check before searching - avoids iterating an empty vector.
    if (markerIds.empty()) {
        hasPrevious_ = false;
        return makeResult(SafetyState::TOOL_NOT_DETECTED);
    }

    // Stage 3: Find expected marker ID 

    // Reject any marker with an unexpected ID - foreign markers in the
    // scene must not be mistaken for the tool.
    int index = -1;
    for (size_t i = 0; i < markerIds.size(); i++) {
        if (markerIds[i] == config_.expectedMarkerId) {
            index = static_cast<int>(i);
            break;
        }
    }

    if (index == -1) {
        hasPrevious_ = false;
        return makeResult(SafetyState::TOOL_NOT_DETECTED);
    }


    // Stage 4: Compute marker centroid 

    // Average of the four corner points - cheaper than full pose estimation
    // and sufficient for zone and speed checks.
    const auto& corners = markerCorners[index];
    cv::Point2f center(0.f, 0.f);
    for (const auto& c : corners)
        center += c;
    center *= 0.25f;


    // Stage 5: Safe zone check 

    if (center.x < config_.safeZoneXMin ||
        center.x > config_.safeZoneXMax ||
        center.y < config_.safeZoneYMin ||
        center.y > config_.safeZoneYMax)
    {
        hasPrevious_ = false;
        return makeResult(SafetyState::OUTSIDE_ALLOWED_ZONE);
    }


    // Stage 6: Speed check 

    if (hasPrevious_) {
        // Pixel displacement between this frame and the last valid detection.
        const float displacement = cv::norm(center - previousPosition_);

        // Convert to pixels/second using fixed 30 FPS period.
        // This matches maxSpeed units defined in VisionConfig.
        // Note: a fixed period is used here for simplicity — could be improved
        // by computing actual elapsed time from captureTimestamp.
        constexpr float framePeriodSeconds = 1.0f / 30.0f;
        const float speed = displacement / framePeriodSeconds;

        if (speed > config_.maxSpeed) {
            hasPrevious_ = false;
            return makeResult(SafetyState::EXCESSIVE_SPEED);
        }
    }


    // Stage 7: Orientation check 

    // Angle computed from corner[0] (top-left) to corner[1] (top-right).
    // atan2 returns radians — converted to degrees for config comparison.
    // const float dx = corners[1].x - corners[0].x;
    // const float dy = corners[1].y - corners[0].y;
    // const float angleDeg = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));

    // if (angleDeg < config_.orientationMin || angleDeg > config_.orientationMax) {
    //    hasPrevious_ = false;
    //    return makeResult(SafetyState::INVALID_ORIENTATION);
    //}

    // All checks passed 

    // Update inter-frame state for next call's speed calculation.
    previousPosition_ = center;
    hasPrevious_ = true;

    return makeResult(SafetyState::SAFE);
}