#include "CameraCapture.hpp"
#include <chrono>
#include <functional>
#include <iostream>
#include <string>

namespace {

std::string buildLibcameraPipeline() {
    // Keep a small, explicit pipeline for Pi/libcamera + OpenCV appsink.
    return "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 "
           "! videoconvert ! video/x-raw,format=BGR ! "
           "appsink drop=true max-buffers=1 sync=false";
}

bool tryOpen(cv::VideoCapture& cap,
             const std::string& label,
             const std::function<bool()>& opener) {
    if (!opener()) {
        std::cerr << "[CameraCapture] Open failed via " << label << std::endl;
        return false;
    }

    std::cout << "[CameraCapture] Opened via " << label;
    if (cap.isOpened()) {
        std::cout << " (backend: " << cap.getBackendName() << ")";
    }
    std::cout << std::endl;
    return true;
}

}  // namespace

CameraCapture::CameraCapture(int camera_index) {
    const std::string libcamera_pipeline = buildLibcameraPipeline();

    const bool opened =
        tryOpen(cap, "GStreamer/libcamerasrc", [&]() {
            return cap.open(libcamera_pipeline, cv::CAP_GSTREAMER);
        }) ||
        tryOpen(cap, "V4L2 index " + std::to_string(camera_index), [&]() {
            return cap.open(camera_index, cv::CAP_V4L2);
        }) ||
        tryOpen(cap, "default backend index " + std::to_string(camera_index),
                [&]() { return cap.open(camera_index); });

    if (!opened) {
        std::cerr << "ERROR: Cannot open camera. Tried libcamerasrc, V4L2, and default backend."
                  << std::endl;
    }

    // Sets the internal buffer size to 1 to ensure the latest frame is fetched,
    // preventing the accumulation of delayed frames.
    if (cap.isOpened() && !cap.set(cv::CAP_PROP_BUFFERSIZE, 1)) {
        std::cerr << "[CameraCapture] Warning: CAP_PROP_BUFFERSIZE unsupported by backend."
                  << std::endl;
    }
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
