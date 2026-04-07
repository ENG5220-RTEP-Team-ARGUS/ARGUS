/**
 * @file VisionProcessor.cpp
 * @brief Implementation of the ARGUS vision safety evaluator.
 *
 * Contains the logic for ArUco marker detection and safety evaluation.
 * Implements a six-stage pipeline: detect -> validate ID -> centroid ->
 * zone check -> speed check -> orientation check.
 *
 * @note All expensive one-time setup (dictionary, detector parameters) is
 * performed in the constructor so that process() stays fast and its
 * latency remains predictable and bounded.
 */

#include "VisionProcessor.hpp"
#include <cmath>

// Constructor

/**
 * @brief Constructs the VisionProcessor and initialises all detection resources.
 *
 * All expensive one-time setup is done here - dictionary, detector parameters,
 * and the ArucoDetector itself - so that process() stays fast and its latency
 * remains bounded on every call.
 *
 * @param config Safety thresholds and zone boundaries injected at construction
 *               to satisfy the Dependency Inversion principle. Stored by value
 *               so external changes cannot affect in-flight safety decisions.
 */
VisionProcessor::VisionProcessor(const VisionConfig& config)
    : config_(config),
      // DICT_6X6_250: small dictionary = higher inter-marker distance =
      // fewer false positives. Important in a safety-critical context.
      // Built once here - not per frame - to keep process() latency bounded.
      //
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

// process()

/**
 * @brief Runs the full vision safety pipeline on a single camera frame.
 *
 * Executes a seven-stage pipeline:
 *   1. Detect all ArUco markers in the frame.
 *   2. Early exit if no markers are detected.
 *   3. Find the expected marker ID - reject foreign markers.
 *   4. Compute the marker centroid from its four corners.
 *   5. Safe zone check - centroid must be within configured bounds.
 *   6. Speed check - pixel displacement per second must not exceed maxSpeed.
 *      Uses actual elapsed time derived from captureTimestamp for correctness
 *      under frame jitter (avoids false SAFE results on dropped frames).
 *   7. Orientation check - marker angle must be within configured range.
 *
 * Processing time is measured at each return point and included in the result
 * to support latency budget analysis.
 *
 * @param frame             The camera frame to analyse. Must be non-empty.
 * @param captureTimestamp  The time at which this frame was captured.
 *                          Used to compute actual inter-frame elapsed time
 *                          for the speed check, ensuring correctness under
 *                          variable frame delivery rates on a loaded RPi.
 * @return SafetyResult containing the safety state, timestamp, and
 *         processing time for this frame.
 */
SafetyResult VisionProcessor::process(
    const cv::Mat& frame,
    std::chrono::steady_clock::time_point captureTimestamp)
{
    // Record entry time so processing_time can be measured at each return point.
    const auto startTime = std::chrono::steady_clock::now();

    /**
     * @brief Helper lambda: builds a SafetyResult with correct timestamps and
     * processing_time without duplicating this logic at every return point.
     */
    auto makeResult = [&](SafetyState state) -> SafetyResult {
        const auto now = std::chrono::steady_clock::now();
        return SafetyResult{
            state,
            now,
            std::chrono::duration_cast<std::chrono::microseconds>(now - startTime)
        };
    };

    // Stage 1: Detect all markers in frame 
    //
    // markerIds:     OpenCV fills this with the ID of each detected marker.
    // markerCorners: Each detected marker gets its own inner list of four
    //                Point2f corners (top-left, top-right, bottom-right,
    //                bottom-left).
    // rejected_:     Required by the OpenCV API but never inspected here.
    //                Promoted to a member variable (see VisionProcessor.hpp)
    //                to avoid a heap allocation on every frame.

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // OpenCV analyses the frame - thresholds it, finds square contours,
    // reads the binary pattern inside each square — and populates
    // markerCorners and markerIds with whatever it finds.
    detector_.detectMarkers(frame, markerCorners, markerIds, rejected_);

    // Stage 2: Early exit if nothing detected 
    //
    // Quick empty check before searching — avoids iterating an empty vector.
    if (markerIds.empty()) {
        hasPrevious_ = false;
        return makeResult(SafetyState::TOOL_NOT_DETECTED);
    }

    // Stage 3: Find expected marker ID 
    //
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
    //
    // Average of the four corner points - cheaper than full pose estimation
    // and sufficient for zone and speed checks.
    const auto& corners = markerCorners[index];
    cv::Point2f center(0.f, 0.f);
    for (const auto& c : corners)
        center += c;
    center *= 0.25f;

    // Stage 5: Safe zone check 
    //
    // Centroid must lie within the configured rectangular safe zone.
    // Any violation immediately returns OUTSIDE_ALLOWED_ZONE.
    if (center.x < config_.safeZoneXMin ||
        center.x > config_.safeZoneXMax ||
        center.y < config_.safeZoneYMin ||
        center.y > config_.safeZoneYMax)
    {
        hasPrevious_ = false;
        return makeResult(SafetyState::OUTSIDE_ALLOWED_ZONE);
    }

    // Stage 6: Speed check
    //
    // Computes pixels/second using the ACTUAL elapsed time between the
    // previous frame and this one, derived from captureTimestamp.
    //
    // Rationale: a hardcoded 1/30 s period is incorrect under frame jitter -
    // on a loaded RPi, frames can arrive 20–60 ms apart. If two frames arrive
    // 66 ms apart (one dropped), a hardcoded period halves the computed speed,
    // meaning a tool at 2× the safe limit appears safe. Using actual elapsed
    // time prevents this class of false-safe result.
    if (hasPrevious_) {
        const float displacement = cv::norm(center - previousPosition_);

        // Compute actual elapsed time since last valid frame.
        // Falls back to 1/30 s on the very first frame (lastTimestamp_ is
        // zero-initialised) to avoid division by zero.
        const float actualPeriodSeconds =
            (lastTimestamp_.time_since_epoch().count() > 0)
            ? std::chrono::duration<float>(captureTimestamp - lastTimestamp_).count()
            : (1.0f / 30.0f);

        const float speed = (actualPeriodSeconds > 0.0f)
            ? displacement / actualPeriodSeconds
            : 0.0f;

        if (speed > config_.maxSpeed) {
            hasPrevious_ = false;
            lastTimestamp_ = captureTimestamp;
            return makeResult(SafetyState::EXCESSIVE_SPEED);
        }
    }

    // Update timestamp for next frame's speed calculation.
    lastTimestamp_ = captureTimestamp;

    // Stage 7: Orientation check 
    //
    // Angle computed from corner[0] (top-left) to corner[1] (top-right).
    // std::atan2 returns radians — converted to degrees for config comparison.
    // Returns INVALID_ORIENTATION if the marker angle falls outside the
    // configured [orientationMin, orientationMax] range.
    {
        const float dx = corners[1].x - corners[0].x;
        const float dy = corners[1].y - corners[0].y;
        const float angleDeg = std::atan2(dy, dx) * (180.0f / static_cast<float>(M_PI));

        if (angleDeg < config_.orientationMin || angleDeg > config_.orientationMax) {
            hasPrevious_ = false;
            return makeResult(SafetyState::INVALID_ORIENTATION);
        }
    }

    // All checks passed 
    // Update inter-frame state for next call's speed calculation.
    previousPosition_ = center;
    hasPrevious_ = true;

    return makeResult(SafetyState::SAFE);
}