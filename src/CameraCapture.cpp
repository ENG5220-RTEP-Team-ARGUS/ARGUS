#include "CameraCapture.hpp"
#include <chrono>
#include <iostream>

CameraCapture::CameraCapture(int camera_index) {
    cap.open(camera_index);

    if (!cap.isOpened()) {
        std::cerr << "ERROR: Cannot open camera index " << camera_index << std::endl;
    }

    // Sets the internal buffer size to 1 to ensure the latest frame is fetched,
    // preventing the accumulation of delayed frames.
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
}

CameraCapture::~CameraCapture() {
    if (cap.isOpened()) {
        cap.release();
    }
}

bool CameraCapture::waitForNextFrame(FrameEvent& output_event) {
    if (!cap.isOpened()) return false;

    // Blocks the thread until the hardware provides the next frame.
    cap >> output_event.image_data;

    // Checks for valid frame data.
    if (output_event.image_data.empty()) {
        std::cerr << "WARNING: Empty frame captured." << std::endl;
        return false;
    }

    // Captures the current system time immediately after the frame arrives.
    auto now = std::chrono::system_clock::now();
    output_event.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();

    return true;
}
