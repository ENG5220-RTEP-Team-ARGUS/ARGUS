#pragma once

/*
 VisionProcessor.hpp
 Stateless-per-frame vision safety evaluator for the ARGUS pipeline.
 
 VisionProcessor is responsible for one thing only: given a camera frame
 and its capture timestamp, detect the expected ArUco marker and evaluate
 whether the current state of the tool is safe.
 
 Design note (Single Responsibility Principle):
 This class does exactly one thing - vision-based safety evaluation.
 All tunable parameters are injected via VisionConfig at construction,
 satisfying the Open/Closed Principle: behaviour can be changed by
 supplying a different config without modifying this class.

 Deliberately decoupled from camera capture, threading, state machine
 logic and logging, making it fully deterministic and unit-testable.
 Tests can call process() directly with a synthetic frame, no camera needed.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>  // Requires OpenCV >= 4.7
#include <chrono>

#include "Types.hpp"
#include "VisionConfig.hpp"


class VisionProcessor {
public:

    //  Constructor
    //  Initialises the ArUco dictionary and detector with
    //  parameters derived from the supplied config.
    //  The config is stored by value so this class owns its
    //  own copy and is not affected by external changes.

    // Construct a VisionProcessor with the given configuration.
    // config: Safety thresholds and zone boundaries. Injected at
    // construction to satisfy Dependency Inversion -
    // this class depends on an abstraction (VisionConfig),
    // not on hardcoded values.
    explicit VisionProcessor(const VisionConfig& config);

    //  Core processing method
    //  Called once per frame by the realtime safety thread.
    //  Must complete within the latency budget (< frame period).
    //  At 30 FPS the budget is ~33ms; target completion < 20ms
    //  to leave headroom for FSM transition and interlock.

    // Process a single camera frame and evaluate tool safety.
    // frame: The raw camera frame to analyse. Must not be empty.
    // timestamp: The time at which this frame was captured by CameraCapture.
    // Used to compute end-to-end pipeline latency in the returned SafetyResult.
    // Return: SafetyResult containing the safety decision, result timestamp and processing duration.
    SafetyResult process(
        const cv::Mat& frame,
        std::chrono::steady_clock::time_point timestamp);


private:

    // Configuration (injected at construction)

    // Owns a copy of the config so external changes cannot
    // affect in-flight processing decisions.
    VisionConfig config_;

    //  ArUco detection members
    //  Initialised once in the constructor to avoid repeated
    //  heap allocation on every frame - keeps process() fast
    //  and processing time predictable.

    // The ArUco marker dictionary to detect against.
    // Determined by config and created once at construction.
    // after
    // TODO: verify OpenCV version on Pi - if < 4.7 revert to cv::Ptr<cv::aruco::Dictionary>
    cv::aruco::Dictionary dictionary_;

    // Tunable detector parameters (thresholding, contour filtering, etc.).
    // Default parameters are used unless overridden via config in future.
    cv::aruco::DetectorParameters detectorParams_;

    // The detector instance. Wraps dictionary + parameters into
    // a single reusable object (OpenCV 4.7+ API).
    cv::aruco::ArucoDetector detector_;

    //  Inter-frame state
    //  Only the previous centroid position is retained between
    //  frames - needed solely to compute marker speed.
    //  All other processing is stateless per-frame.

    // Centroid position of the marker in the previous frame.
    // Used to compute pixel displacement per second for speed check.
    cv::Point2f previousPosition_;

    // Tracks whether a valid previous position exists.
    // False on first frame or after a TOOL_NOT_DETECTED result,
    // preventing an untrue EXCESSIVE_SPEED on the first detection.
    bool hasPrevious_ = false;
};