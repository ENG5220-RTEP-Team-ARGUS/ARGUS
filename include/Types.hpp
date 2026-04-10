/**
 * @file Types.hpp
 * @brief Shared data types passed between ARGUS pipeline components.
 *
 * Defines the core structs and enums passed between components in the
 * ARGUS pipeline. Acts as a shared contract between classes so that each
 * module (CameraCapture, VisionProcessor, GuardianStateMachine, etc.)
 * can communicate without being tightly coupled to one another.
 */

#pragma once

#include <chrono>
#include <opencv2/core.hpp>

/**
 * @brief Outcome of a single vision safety evaluation.
 *
 * Produced by VisionProcessor::process() and consumed by
 * GuardianStateMachine::processFrame() to drive state transitions.
 */
enum class SafetyState {
    /** Marker detected - position, speed, and orientation all within bounds. */
    SAFE,

    /** No marker found in the frame - tool removed, occluded, or camera fault. */
    TOOL_NOT_DETECTED,

    /** Marker centroid lies outside the configured safe zone rectangle. */
    OUTSIDE_ALLOWED_ZONE,

    /** Marker moved faster than config_.maxSpeed between frames. */
    EXCESSIVE_SPEED,

    /** Marker rotation falls outside the configured angular range. */
    INVALID_ORIENTATION,

    DEPTH_EXCEEDED, ///< The forbidden playdough layer colour was detected
                    ///< within the ROI, indicating the tool has penetrated
                    ///< beyond the permitted depth. Triggers an immediate
                    ///< FREEZE_NOW event and retract command via RobotInterlock.
};

/**
 * @brief Output of VisionProcessor::process() for each camera frame.
 *
 * Contains the safety decision and timing information for latency
 * measurement and logging. Processing time is measured inside
 * VisionProcessor and included here to support latency budget analysis.
 */
struct SafetyResult {

    /**
     * @brief The safety decision made for this frame.
     *
     * Evaluated by GuardianStateMachine to determine whether to transition
     * to FROZEN_UNSAFE or remain in SAFE_MONITORING.
     */
    SafetyState state;

    /**
     * @brief The time at which this result was produced (end of processing).
     *
     * Set inside VisionProcessor::process() at each return point so that
     * processing_time can be computed accurately.
     */
    std::chrono::steady_clock::time_point timestamp;

    /**
     * @brief How long VisionProcessor took to process this frame.
     *
     * Measured as: time_point at exit minus time_point at entry of process().
     * Used to verify the pipeline stays within its latency budget
     * (target: under 20 ms at 30 FPS to leave headroom for FSM and interlock).
     */
    std::chrono::microseconds processing_time;
};


/**
 * @brief Camera frame bundled with its capture timestamp.
 *
 * Produced by CameraCapture and passed into VisionProcessor::process().
 * The timestamp is set at the moment of capture (not inside process())
 * to enable true end-to-end latency measurement across the pipeline.
 */
struct FrameData {

    /**
     * @brief The raw image frame from the camera.
     */
    cv::Mat frame;

    /**
     * @brief The time at which this frame was captured by CameraCapture.
     *
     * Set at the moment the blocking frame read returns in CameraCapture.
     * Passed through to VisionProcessor to compute actual inter-frame
     * elapsed time for the speed check - ensuring correctness under
     * variable frame delivery rates on a loaded Raspberry Pi.
     */
    std::chrono::steady_clock::time_point timestamp;
};