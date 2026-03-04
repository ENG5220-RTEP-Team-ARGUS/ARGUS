#pragma once // Prevents the header file from being included more than once, avoiding redefinition errors

/**
 * @file Types.hpp
 * @brief Shared data types for the ARGUS vision safety pipeline.
 *
 * This file defines the core structs and enums that are passed between
 * components in the system. It acts as a shared contract between classes
 * so that each module (CameraCapture, VisionProcessor, GuardianStateMachine,
 * etc.) can communicate without being tightly coupled to one another.
 *
 */

#include <chrono>
#include <opencv2/core.hpp>

//  SafetyState
//  Represents the outcome of a single vision safety evaluation.
//  Produced by VisionProcessor and consumed by GuardianStateMachine.

enum class SafetyState {

    // Marker detected, position and orientation are within allowed bounds,
    // and speed is below the maximum threshold. System is clear to operate.
    SAFE,

    // No ArUco marker was found in the region of interest.
    // Could indicate tool removed, occluded or camera failure.
    TOOL_NOT_DETECTED,

    // Marker was detected but its centroid falls outside the
    // defined safe zone boundary.
    OUTSIDE_ALLOWED_ZONE,

    // Marker centroid moved faster than the permitted speed
    // between the previous and current frame.
    EXCESSIVE_SPEED,

    // Marker orientation (angle) is outside the permitted range.
    // Could indicate unexpected tool rotation or tilt.
    INVALID_ORIENTATION
};


//  SafetyResult
//  The output produced by VisionProcessor for each processed frame.
//  Contains the safety decision and timing information for
//  latency measurement and logging.

struct SafetyResult {

    // The safety decision made for this frame.
    SafetyState state;

    // The time at which this result was produced (end of processing).
    // Set inside VisionProcessor::processFrame().
    std::chrono::steady_clock::time_point timestamp;

    // How long VisionProcessor took to process this frame.
    // Used to verify the pipeline stays within its latency budget.
    // Measured as: time_point at exit - time_point at entry of processFrame().
    std::chrono::microseconds processing_time;
};

//  FrameData
//  A camera frame bundled with the timestamp of when it was captured.
//  Produced by CameraCapture and passed into VisionProcessor.
//
//  Carrying the capture timestamp here (rather than generating one
//  inside VisionProcessor) allows total end-to-end pipeline latency
//  to be measured: from frame capture all the way to safety decision.

struct FrameData {

    /// The raw image frame from the camera.
    cv::Mat frame;

    /// The time at which this frame was captured by CameraCapture.
    /// Set at the moment the blocking frame read returns.
    std::chrono::steady_clock::time_point timestamp;
};