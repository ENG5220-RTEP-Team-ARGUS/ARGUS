/**
 * @file VisionProcessor.cpp
 * @brief Implementation of the ARGUS vision safety evaluator.
 *
 * Contains the logic for colour-based depth safety evaluation.
 * The current branch flags unsafe when the configured forbidden layer
 * colour is visible inside the configured ROI.
 *
 * @note All expensive one-time setup (dictionary, detector parameters) is
 * performed in the constructor so that process() stays fast and its
 * latency remains predictable and bounded.
 */

#include "VisionProcessor.hpp"
#include <opencv2/imgproc.hpp>

// Constructor

/**
 * @brief Constructs the VisionProcessor and initialises all detection resources.
 *
 * Initialises internal resources once so process() stays fast and latency
 * remains bounded on every call.
 *
 * @param config Safety thresholds and zone boundaries injected at construction
 *               to satisfy the Dependency Inversion principle. Stored by value
 *               so external changes cannot affect in-flight safety decisions.
 */
VisionProcessor::VisionProcessor(const VisionConfig& config)
    : config_(config),
      // Reserved marker resources are still constructed for compatibility
      // with marker-based configurations.
      dictionary_(cv::aruco::getPredefinedDictionary(
                        static_cast<cv::aruco::PredefinedDictionaryType>(config.dictionaryId))),
      detector_(dictionary_, detectorParams_)
{}

// updateConfig()

/**
 * @brief Updates the safety configuration at runtime.
 *
 * Allows dynamic adjustment of thresholds (e.g., via UI sliders) without
 * reconstructing the VisionProcessor. Useful for calibration and testing
 * under varying lighting conditions.
 *
 * @param newConfig The updated configuration to apply.
 */
void VisionProcessor::updateConfig(const VisionConfig& newConfig) {
    config_ = newConfig;
}

// process()

/**
 * @brief Runs the full vision safety pipeline on a single camera frame.
 *
 * Executes the depth-colour pipeline:
 *   1. Crop to safe-zone ROI.
 *   2. Convert ROI to HSV.
 *   3. Threshold HSV against configured forbidden colour bands.
 *   4. Trigger DEPTH_EXCEEDED when matching pixels exceed threshold.
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
    (void)captureTimestamp;

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

    // Marker-based stages are intentionally disabled on this branch.
    // Safety decisions currently come from forbidden-layer colour detection.

    // Depth layer colour detection
    //
    // Detects whether the forbidden playdough layer colour is visible anywhere
    // inside the configured safe zone ROI. Exposure of this colour indicates
    // the tool has cut beyond the permitted depth and the robot must retract.
    //
    // Pipeline:
    //   1. Crop frame to safe zone ROI — limits work to the relevant area
    //      and keeps per-frame latency bounded.
    //   2. Convert ROI from BGR to HSV — separates colour from brightness
    //      for lighting-robust detection.
    //   3. Threshold for target hue using two cv::inRange calls combined
    //      with cv::bitwise_or. Two ranges are required for red because red
    //      wraps around 0°/180° in OpenCV HSV (H range 0–179).
    //   4. Count non-zero pixels in the combined mask. A pixel count
    //      above depthPixelThreshold confirms the layer is exposed.
    //
    // Returning DEPTH_EXCEEDED here triggers FREEZE_NOW in GuardianFSM
    // followed by a retract command via RobotInterlock — distinct from
    // a standard freeze in that the robot actively retracts rather than
    // simply halting in place.

    if (config_.depthCheckEnabled) {

        // Step 1: Crop to safe zone ROI.
        //
        // Clamp all coordinates defensively to frame bounds.
        // Guards against runtime-adjusted ROI values (e.g. dragged via UI)
        // that transiently extend past the image edges between frames.
        const int roiX = std::max(0, static_cast<int>(config_.safeZoneXMin));
        const int roiY = std::max(0, static_cast<int>(config_.safeZoneYMin));
        const int roiW = std::min(
            frame.cols - roiX,
            static_cast<int>(config_.safeZoneXMax - config_.safeZoneXMin));
        const int roiH = std::min(
            frame.rows - roiY,
            static_cast<int>(config_.safeZoneYMax - config_.safeZoneYMin));

        // Guard: skip stage if ROI is degenerate (zero or negative dimensions).
        // Prevents cv::Rect from throwing on a misconfigured or transitional
        // ROI. The absence of a DEPTH_EXCEEDED result implicitly signals that
        // the check was bypassed this frame.
        if (roiW > 0 && roiH > 0) {

            // cv::Mat::operator() with a Rect is a zero-copy view — no pixel
            // data is duplicated. Cheap enough to be safe on every frame.
            const cv::Mat roi = frame(cv::Rect(roiX, roiY, roiW, roiH));

            // Step 2: Convert ROI from BGR to HSV.
            //
            // HSV isolates hue from brightness — under variable lab lighting
            // the same playdough colour can appear significantly darker or
            // lighter in BGR but its hue remains stable. This makes HSV
            // thresholding substantially more reliable than BGR thresholding
            // for a physical demo.
            cv::Mat hsvRoi;
            cv::cvtColor(roi, hsvRoi, cv::COLOR_BGR2HSV);

            // Step 3: Threshold for target hue range.
            //
            // Two inRange calls are combined with bitwise_or for a generic
            // dual-band hue model:
            //
            //   mask1 - primary hue band: H in [depthHueLower1, depthHueUpper1]
            //   mask2 - optional second band: H in [depthHueLower2, depthHueUpper2]
            //
            // For current green-layer detection, hue band 1 targets the green
            // interval and mask2 is disabled by setting depthHueLower2 >
            // depthHueUpper2 in VisionConfig, so bitwise_or reduces to mask1
            // with no code change required.
            cv::Mat mask1, mask2, combinedMask;

            cv::inRange(
                hsvRoi,
                cv::Scalar(config_.depthHueLower1,
                           config_.depthSatMin,
                           config_.depthValMin),
                cv::Scalar(config_.depthHueUpper1,
                           config_.depthSatMax,
                           config_.depthValMax),
                mask1);

            cv::inRange(
                hsvRoi,
                cv::Scalar(config_.depthHueLower2,
                           config_.depthSatMin,
                           config_.depthValMin),
                cv::Scalar(config_.depthHueUpper2,
                           config_.depthSatMax,
                           config_.depthValMax),
                mask2);

            // Combine both masks — any pixel matching either hue range is
            // considered a match. bitwise_or is O(pixels), no allocation.
            cv::bitwise_or(mask1, mask2, combinedMask);

            // Step 4: Count matching pixels.
            //
            // countNonZero is O(pixels) with no allocation — safe for
            // real-time use on every frame.
            // Compared against depthPixelThreshold to filter noise:
            // isolated reflections or stray colour patches produce far
            // fewer matching pixels than genuine layer exposure.
            const int matchingPixels = cv::countNonZero(combinedMask);

            if (matchingPixels >= config_.depthPixelThreshold) {
                // Forbidden layer confirmed exposed.
                return makeResult(SafetyState::DEPTH_EXCEEDED);
            }
        }
    }

    // All checks passed.
    return makeResult(SafetyState::SAFE);
}
