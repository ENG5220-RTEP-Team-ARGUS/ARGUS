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

    // Depth layer colour detection
    //
    // The forbidden layer is a specific playdough colour (blue) placed
    // beneath the permitted cutting layers. When this colour becomes visible
    // inside the ROI the tool has exceeded its permitted depth and the robot
    // must freeze and retract immediately.
    //
    // HSV is used in preference to BGR because it separates colour (hue)
    // from brightness (value), making detection robust under variable
    // lighting conditions — critical for a physical hardware demo under
    // lab fluorescent lighting.
    //
    // OpenCV HSV channel ranges:
    //   H (hue):        0 – 179   (half of the 360° colour wheel)
    //   S (saturation): 0 – 255
    //   V (value):      0 – 255
    //
    // Blue does not wrap around the HSV wheel, so only range 1 is needed.
    // Range 2 is disabled by setting depthHueLower2 > depthHueUpper2,
    // which causes cv::inRange to produce an empty mask2 so the
    // bitwise_or in Stage 8 reduces to mask1 alone.

    int depthHueLower1 = 100;   ///< Lower hue bound for blue.
    int depthHueUpper1 = 130;   ///< Upper hue bound for blue.

    int depthHueLower2 = 255;   ///< Disabled - lower > upper produces empty mask2.
    int depthHueUpper2 = 0;     ///< Disabled - see depthHueLower2 above.

    int depthSatMin = 100;      ///< Minimum saturation threshold.
                                ///< Filters washed-out or near-grey pixels
                                ///< that share the target hue by coincidence.

    int depthSatMax = 255;      ///< Maximum saturation threshold.

    int depthValMin = 50;       ///< Minimum value (brightness) threshold.
                                ///< Filters very dark pixels that appear
                                ///< to match the hue under shadow conditions.

    int depthValMax = 255;      ///< Maximum value threshold.

    /**
     * @brief Minimum number of HSV-matching pixels required to confirm
     *        the forbidden layer is exposed.
     *
     * Acts as a noise gate - isolated reflections, stray colour patches,
     * or single-pixel sensor noise will not trigger a freeze. If false
     * positives occur during testing, increase this value. If the system
     * is slow to detect a genuine exposure, decrease it.
     *
     * Recommended starting point: 500 pixels at 640x480 resolution.
     */
    int depthPixelThreshold = 500;

    /**
     * @brief Runtime enable/disable flag for Stage 8 colour detection.
     *
     * Allows the depth check to be toggled from the UI without
     * recompiling. Satisfies the brief requirement for mouse-adjustable
     * parameters and supports testing of the remaining pipeline stages
     * in isolation.
     *
     * true  = Stage 8 active (default, production behaviour).
     * false = Stage 8 bypassed (testing / calibration mode).
     */
    bool depthCheckEnabled = true;
};