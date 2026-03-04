#pragma once // Prevents the header file from being included more than once, avoiding redefinition errors

/*
Types.hpp
Shared data types passed between ARGUS pipeline components
 
This file defines the core structs and enums that are passed between
components in the system. It acts as a shared contract between classes
so that each module (CameraCapture, VisionProcessor, GuardianStateMachine,
etc.) can communicate without being tightly coupled to one another.
 
*/

#include <chrono>
#include <opencv2/core.hpp>

//  SafetyState
//  Outcome of a single vision safety evaluation.
//  Produced by VisionProcessor, consumed by GuardianStateMachine.

enum class SafetyState {
    SAFE,                  // Marker detected, position/orientation/speed all within bounds.
    TOOL_NOT_DETECTED,     // No marker found in ROI — tool removed, occluded, or camera fault.
    OUTSIDE_ALLOWED_ZONE,  // Marker centroid outside the defined safe zone rectangle.
    EXCESSIVE_SPEED,       // Marker moved faster than maxSpeed between frames.
    INVALID_ORIENTATION    // Marker rotation outside the permitted angular range.
};


//  SafetyResult
//  Output of VisionProcessor::process() for each frame.
//  Contains the safety decision and timing information for latency measurement and logging.

struct SafetyResult {

    // The safety decision made for this frame.
    SafetyState state;

    // The time at which this result was produced (end of processing).
    // Set inside VisionProcessor::processFrame().
    std::chrono::steady_clock::time_point timestamp;

    // How long VisionProcessor took to process this frame. Used to verify the pipeline stays
    // within its latency budget.
    // Measured as: time_point at exit - time_point at entry of processFrame().
    std::chrono::microseconds processing_time;
};

//  FrameData
//  Camera frame bundled with its capture timestamp.
//  Produced by CameraCapture, passed into VisionProcessor.
//  Timestamp is set at capture (not inside process()) to enable
//  true end-to-end latency measurement across the pipeline.

struct FrameData {

    /// The raw image frame from the camera.
    cv::Mat frame;

    /// The time at which this frame was captured by CameraCapture.
    /// Set at the moment the blocking frame read returns.
    std::chrono::steady_clock::time_point timestamp;
};