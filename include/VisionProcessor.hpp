/**
 * @file VisionProcessor.hpp
 * @brief Vision safety evaluator for the ARGUS pipeline.
 *
 * VisionProcessor is responsible for one thing only: given a camera frame
 * and its capture timestamp, evaluate whether the forbidden depth-layer
 * colour is visible in the configured ROI.
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
 * (the realtime safety thread).
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
     * Initialises internal resources once at construction so that process()
     * stays fast and latency remains predictable and bounded on every call.
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
     */
    explicit VisionProcessor(const VisionConfig& config);

    // Core Interface 

    /**
     * @brief Processes a single camera frame and evaluates tool safety.
     *
     * Executes a depth-colour safety pipeline per frame:
     *   1. Crop frame to the configured safe-zone ROI.
     *   2. Convert ROI to HSV colour space.
     *   3. Apply configured hue/saturation/value thresholds.
     *   4. Trigger DEPTH_EXCEEDED when matching pixels exceed threshold.
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
     *         - The safety state (SAFE or DEPTH_EXCEEDED on this branch).
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

    // Reserved marker-detection members (currently inactive on this branch)
    //
    // Kept only for compatibility with marker-based configurations.

    /**
     * @brief Reserved marker dictionary.
     *
     * Determined by config_.dictionaryId and created once at construction.
     * DICT_6X6_250 provides high inter-marker distance, reducing false
     * positives — important in a safety-critical context.
     *
     */
    cv::aruco::Dictionary dictionary_;

    /**
     * @brief Reserved marker detector parameters.
     *
     * Default parameters are used unless overridden via config in future.
     * Constructed once — not per frame.
     */
    cv::aruco::DetectorParameters detectorParams_;

    /**
     * @brief Reserved marker detector instance.
     *
     * Wraps dictionary_ and detectorParams_ into a single reusable object
     * (OpenCV 4.7+ API). Constructed once at construction to keep
     * process() latency bounded.
     */
    cv::aruco::ArucoDetector detector_;

    // Reserved inter-frame members from marker pipeline (currently inactive)

    /**
     * @brief Reserved previous marker centroid.
     */
    cv::Point2f previousPosition_;

    /**
     * @brief Reserved previous-marker availability flag.
     */
    bool hasPrevious_ = false;

    /**
     * @brief Reserved previous timestamp from marker-speed checks.
     */
    std::chrono::steady_clock::time_point lastTimestamp_{};

    /**
     * @brief Reserved buffer for rejected marker candidates.
     */
    mutable std::vector<std::vector<cv::Point2f>> rejected_;
};
