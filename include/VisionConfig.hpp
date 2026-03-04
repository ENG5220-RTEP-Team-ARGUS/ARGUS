#pragma once

/*
 VisionConfig.hpp
 Configuration parameters for the VisionProcessor.
 
 This struct holds all tunable parameters that control how the
 VisionProcessor evaluates safety. By separating configuration
 from algorithm logic, the VisionProcessor remains deterministic
 and testable and different configs can be injected without touching
 any processing code.
 
 In production, these values would be loaded from config/vision_config.yaml
 at startup rather than relying on the defaults defined here.
 
 No logic lives here, only plain configuration data.
*/

//  VisionConfig
//  Injected into VisionProcessor at construction time.
//  All safety thresholds and zone boundaries are defined here.

struct VisionConfig {

    //  Marker identification

    // The ArUco marker ID that the system expects to see.
    // Any detected marker with a different ID will be treated
    // as TOOL_NOT_DETECTED to prevent spoofing by foreign markers.
    int expectedMarkerId = 23;

    //  Speed threshold

    // Maximum permitted speed of the marker centroid between
    // consecutive frames, in pixels per second.
    // Exceeding this triggers SafetyState::EXCESSIVE_SPEED.
    // Value should be calibrated against the physical robot's
    // maximum safe operating speed projected into image space.
    float maxSpeed = 200.0f;

    //  Safe zone boundary (pixels in camera image space)
    //  Defines a rectangular region within which the marker
    //  centroid must remain for the system to report SAFE.
    //  Coordinates are relative to the full camera frame,
    //  before any ROI crop is applied.

    // Left boundary of the safe zone (pixels).
    int safeZoneXMin = 100;

    // Right boundary of the safe zone (pixels).
    int safeZoneXMax = 500;

    // Top boundary of the safe zone (pixels).
    int safeZoneYMin = 100;

    /// Bottom boundary of the safe zone (pixels).
    int safeZoneYMax = 400;


    //  Orientation limits (degrees)
    //  Defines the permitted angular range of the marker.
    //  Angles are computed from the marker corner positions
    //  and represent rotation in the image plane.

    // Minimum permitted marker orientation angle in degrees.
    float orientationMin = -45.0f;

    // Maximum permitted marker orientation angle in degrees.
    float orientationMax =  45.0f;
};