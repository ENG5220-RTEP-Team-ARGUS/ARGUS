#pragma once

/*
 VisionConfig.hpp
 Configuration parameters for the VisionProcessor.
 
 This struct holds all tunable parameters that control how the
 VisionProcessor evaluates safety. By separating configuration
 from algorithm logic, the VisionProcessor remains deterministic
 and testable and different configs can be used without touching
 any processing code.
 
 In production, these values would be loaded from config/vision_config.yaml
 at startup rather than relying on the defaults defined here.
*/

//  VisionConfig
//  Implemented into VisionProcessor at construction time.
//  All safety thresholds and zone boundaries are defined here.

struct VisionConfig {

    //  Marker identification

    // The ArUco marker ID that the system expects to see.
    // Any detected marker with a different ID will be treated
    // as TOOL_NOT_DETECTED to prevent spoofing by foreign markers.
    int expectedMarkerId = 23;

    //  Speed threshold

    // Max permitted centroid displacement in pixels/second.
    // Exceeding this triggers SafetyState::EXCESSIVE_SPEED.
    // Calibrate against the robot's max safe speed projected into image space.
    float maxSpeed = 200.0f;

    //  Safe zone (pixels, in full camera frame coordinates)
    //  Defines a rectangular region within which the marker
    //  centroid must remain for the system to report SAFE.
    //  Violation triggers SafetyState::OUTSIDE_ALLOWED_ZONE.

    // Left boundary of the safe zone (pixels).
    int safeZoneXMin = 100;

    // Right boundary of the safe zone (pixels).
    int safeZoneXMax = 500;

    // Top boundary of the safe zone (pixels).
    int safeZoneYMin = 100;

    // Bottom boundary of the safe zone (pixels).
    int safeZoneYMax = 400;


    //  Orientation limits (degrees, in image plane)
    //  Defines the permitted angular range of the marker.
    //  Angles are computed from the marker corner positions
    //  and represent rotation in the image plane.
    //  Violation triggers SafetyState::INVALID_ORIENTATION.

    // Minimum permitted marker orientation angle in degrees.
    float orientationMin = -45.0f;

    // Maximum permitted marker orientation angle in degrees.
    float orientationMax =  45.0f;

    // ArUco dictionary used for marker detection.
    // DICT_6X6_250 by default — change here without touching VisionProcessor.
    int dictionaryId = cv::aruco::DICT_6X6_250;
};