#pragma once

#include <opencv2/opencv.hpp>

// Structure representing the output data: frame object and its timestamp.
struct FrameEvent {
    cv::Mat image_data;
    long long timestamp_ms;
};

// Class for the camera capture module.
class CameraCapture {
private:
    cv::VideoCapture cap;

public:
    // Initializes the camera using the specified device index.
    CameraCapture(int camera_index = 0);

    // Releases camera hardware resources.
    ~CameraCapture();

    // Blocks execution until a new frame is available.
    // Populates output_event with the frame and a timestamp.
    // Returns true on success, false on failure or empty frame.
    bool waitForNextFrame(FrameEvent& output_event);
};