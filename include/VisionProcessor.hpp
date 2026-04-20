/**
 * @file VisionProcessor.hpp
 * @brief Vision safety evaluator for the ARGUS pipeline.
 *
 * VisionProcessor is responsible for one thing only: given a camera frame
 * and its capture timestamp, detect the expected ArUco marker and evaluate
 * whether the current state of the tool is safe.
 *
 * @par Design - Single Responsibility Principle
 * This class does exactly one thing: vision-based safety evaluation.
 * All tunable parameters are injected via VisionConfig at construction,
 * satisfying the Open/Closed Principle - behaviour can be changed by
 * supplying a different config without modifying this class.
 *
 * @par Decoupling
 * Deliberately decoupled from camera capture, threading, state machine
 * logic and logging, making it fully deterministic and unit-testable.
 * Tests can call process() directly with a synthetic frame — no camera needed.
 *
 * @par Thread Safety
 * Not thread-safe. process() must only be called from a single thread
 * (the realtime safety thread). Inter-frame state (previousPosition_,
 * hasPrevious_, lastTimestamp_) is plain - no atomics required as long
 * as this constraint is respected.
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>  // Requires OpenCV >= 4.7
#include <chrono>
#include <vector>

#include "Types.hpp"
#include "VisionConfig.hpp"


class VisionProcessor {
public:

    // Construction 

    /**
     * @brief Constructs a VisionProcessor with the given safety configuration.
     *
     * Initialises the ArUco dictionary, detector parameters, and ArucoDetector
     * once at construction so that process() stays fast and its latency remains
     * predictable and bounded on every call.
     *
     * The config is stored by value so this class owns its own copy and is
     * not affected by any external changes after construction — satisfying
     * the Dependency Inversion Principle (depends on VisionConfig abstraction,
     * not hardcoded values).
     *
     * @param config Safety thresholds and zone boundaries injected at
     *               construction. Stored by value to prevent external
     *               modifications from affecting in-flight decisions.
     *
     * @note TODO: verify OpenCV version on Raspberry Pi before deployment.
     *       Current code uses OpenCV 4.7+ API (Dictionary by value).
     *       If Pi has OpenCV < 4.7, revert dictionary_ to cv::Ptr<cv::aruco::Dictionary>.
     *       Check with: pkg-config --modversion opencv4
     */
    explicit VisionProcessor(const VisionConfig& config);

    // Core Interface 

    /**
     * @brief Processes a single camera frame and evaluates tool safety.
     *
     * Executes a seven-stage pipeline per frame:
     *   1. Detect all ArUco markers in the frame.
     *   2. Early exit if no markers are detected (TOOL_NOT_DETECTED).
     *   3. Find the expected marker ID - reject foreign markers.
     *   4. Compute the marker centroid from its four corner points.
     *   5. Safe zone check - centroid must lie within configured bounds.
     *   6. Speed check - pixel displacement per second must not exceed
     *      config_.maxSpeed. Uses actual elapsed time from captureTimestamp
     *      to correctly handle variable frame delivery rates (frame jitter).
     *   7. Orientation check - marker angle must be within configured range.
     *
     * Must complete within the latency budget (less than one frame period).
     * At 30 FPS the budget is ~33 ms; target completion under 20 ms to leave
     * headroom for FSM transition and interlock signalling.
     *
     * @param frame            The raw camera frame to analyse. Must not be empty.
     * @param captureTimestamp The time at which this frame was captured.
     *                         Used to compute the actual inter-frame elapsed
     *                         time for the speed check — prevents false SAFE
     *                         results caused by dropped frames under load.
     *
     * @return SafetyResult containing:
     *         - The safety state (SAFE, TOOL_NOT_DETECTED, OUTSIDE_ALLOWED_ZONE,
     *           EXCESSIVE_SPEED, or INVALID_ORIENTATION).
     *         - The timestamp at which the result was produced.
     *         - The processing duration for latency budget analysis.
     */
    SafetyResult process(
        const cv::Mat& frame,
        std::chrono::steady_clock::time_point captureTimestamp);

    /**
     * @brief Updates the safety configuration at runtime.
     *
     * Allows dynamic adjustment of thresholds (e.g., via UI sliders) without
     * reconstructing the VisionProcessor. Useful for calibration and testing
     * under varying lighting conditions.
     *
     * @param newConfig The updated configuration to apply.
     */
    void updateConfig(const VisionConfig& newConfig);

private:

    // Configuration

    /**
     * @brief Safety thresholds and zone boundaries.
     *
     * Stored by value so external changes cannot affect in-flight
     * processing decisions after construction.
     */
    VisionConfig config_;

    // ArUco Detection Members
    //
    // All detection resources are initialised once in the constructor to avoid
    // repeated heap allocation on every frame — keeps process() fast and
    // processing time predictable.

    /**
     * @brief The ArUco marker dictionary used for detection.
     *
     * Determined by config_.dictionaryId and created once at construction.
     * DICT_6X6_250 provides high inter-marker distance, reducing false
     * positives — important in a safety-critical context.
     *
     * @note If OpenCV < 4.7, revert to cv::Ptr<cv::aruco::Dictionary>.
     */
    cv::aruco::Dictionary dictionary_;

    /**
     * @brief Tunable detector parameters (thresholding, contour filtering etc.).
     *
     * Default parameters are used unless overridden via config in future.
     * Constructed once — not per frame.
     */
    cv::aruco::DetectorParameters detectorParams_;

    /**
     * @brief The ArUco detector instance.
     *
     * Wraps dictionary_ and detectorParams_ into a single reusable object
     * (OpenCV 4.7+ API). Constructed once at construction to keep
     * process() latency bounded.
     */
    cv::aruco::ArucoDetector detector_;

    // Inter-frame State
    //
    // Only the minimum state required between frames is retained.
    // All other processing is stateless per-frame.

    /**
     * @brief Centroid position of the marker in the previous valid frame.
     *
     * Used to compute pixel displacement per second for the speed check.
     * Only valid when hasPrevious_ is true.
     */
    cv::Point2f previousPosition_;

    /**
     * @brief Whether a valid previous position exists for speed computation.
     *
     * Set to false on first frame and after any non-SAFE result to prevent
     * a false EXCESSIVE_SPEED on the first detection after a gap.
     */
    bool hasPrevious_ = false;

    /**
     * @brief Timestamp of the last successfully processed frame.
     *
     * Used to compute the actual inter-frame elapsed time in the speed check,
     * ensuring correctness under variable frame delivery rates on a loaded RPi.
     * Zero-initialised — the speed check falls back to 1/30 s on the first frame.
     */
    std::chrono::steady_clock::time_point lastTimestamp_{};

    /**
     * @brief Reusable buffer for ArUco marker candidates rejected during detection.
     *
     * Required by the OpenCV detectMarkers() API but never inspected here.
     * Promoted to a member variable to avoid a heap allocation on every frame
     * at 30 FPS - cleared implicitly by detectMarkers() on each call.
     */
    mutable std::vector<std::vector<cv::Point2f>> rejected_;
};