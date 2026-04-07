/**
 * @file VisionConfig.hpp
 * @brief Configuration parameters for the VisionProcessor.
 *
 * This struct holds all tunable parameters that control how the
 * VisionProcessor evaluates safety. By separating configuration from
 * algorithm logic, the VisionProcessor remains deterministic and testable,
 * and different configurations can be applied without touching any
 * processing code - satisfying the Open/Closed Principle.
 *
 * @note In production, these values should be loaded from
 *       config/vision_config.yaml at startup rather than relying on
 *       the defaults defined here.
 */

#pragma once

#include <opencv2/objdetect/aruco_detector.hpp>

/**
 * @brief All tunable safety thresholds and zone boundaries for VisionProcessor.
 *
 * Injected into VisionProcessor at construction time via the constructor
 * parameter. Stored by value inside VisionProcessor so external changes
 * after construction cannot affect in-flight safety decisions.
 *
 * @par Usage
 * @code
 * VisionConfig config;
 * config.expectedMarkerId = 42;
 * config.maxSpeed = 150.0f;
 * VisionProcessor vp(config);
 * @endcode
 */
struct VisionConfig {

    // Marker Identification 

    /**
     * @brief The ArUco marker ID the system expects to detect.
     *
     * Any detected marker with a different ID is treated as
     * TOOL_NOT_DETECTED to prevent spoofing by foreign markers in the scene.
     * Default: 23.
     */
    int expectedMarkerId = 23;

    // Speed Threshold 

    /**
     * @brief Maximum permitted marker centroid displacement in pixels/second.
     *
     * Exceeding this threshold triggers SafetyState::EXCESSIVE_SPEED.
     * Calibrate against the robot's maximum safe operational speed
     * projected into image space at the working distance.
     * Default: 200.0 px/s.
     */
    float maxSpeed = 200.0f;

    // Safe Zone (pixels, full camera frame coordinates) 
    //
    // Defines a rectangular region within which the marker centroid must
    // remain for the system to report SAFE. Any centroid outside this
    // rectangle triggers SafetyState::OUTSIDE_ALLOWED_ZONE.

    /**
     * @brief Left boundary of the safe zone in pixels.
     *
     * Centroid x must be >= safeZoneXMin. Default: 100.
     */
    int safeZoneXMin = 100;

    /**
     * @brief Right boundary of the safe zone in pixels.
     *
     * Centroid x must be <= safeZoneXMax. Default: 500.
     */
    int safeZoneXMax = 500;

    /**
     * @brief Top boundary of the safe zone in pixels.
     *
     * Centroid y must be >= safeZoneYMin. Default: 100.
     */
    int safeZoneYMin = 100;

    /**
     * @brief Bottom boundary of the safe zone in pixels.
     *
     * Centroid y must be <= safeZoneYMax. Default: 400.
     */
    int safeZoneYMax = 400;

    // Orientation Limits (degrees, image plane)
    //
    // Defines the permitted angular range of the marker in the image plane.
    // Angles are computed from the marker corner positions using atan2.
    // Any angle outside [orientationMin, orientationMax] triggers
    // SafetyState::INVALID_ORIENTATION.

    /**
     * @brief Minimum permitted marker orientation angle in degrees.
     *
     * Computed from corner[0] to corner[1] using atan2.
     * Default: -45.0 degrees.
     */
    float orientationMin = -45.0f;

    /**
     * @brief Maximum permitted marker orientation angle in degrees.
     *
     * Computed from corner[0] to corner[1] using atan2.
     * Default: 45.0 degrees.
     */
    float orientationMax = 45.0f;

    // ArUco Dictionary

    /**
     * @brief ArUco dictionary used for marker detection.
     *
     * DICT_6X6_250 by default - provides high inter-marker Hamming distance,
     * reducing false positives in safety-critical contexts.
     * Change here to switch dictionary without modifying VisionProcessor.
     */
    int dictionaryId = cv::aruco::DICT_6X6_250;
};