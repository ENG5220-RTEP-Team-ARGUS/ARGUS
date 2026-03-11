#include "CameraCapture.hpp"
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::vector<std::string> buildLibcameraPipelines() {
    // Ordered from stricter caps to more permissive fallback.
    return {
        "libcamerasrc ! video/x-raw,format=NV12,width=640,height=480,framerate=30/1 "
        "! videoconvert ! appsink drop=true max-buffers=1 sync=false",
        "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 "
        "! videoconvert ! appsink drop=true max-buffers=1 sync=false",
        "libcamerasrc ! videoconvert ! appsink drop=true max-buffers=1 sync=false"};
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

bool isLibcamerifyActive() {
    const char* ld_preload = std::getenv("LD_PRELOAD");
    if (ld_preload == nullptr) {
        return false;
    }

    const std::string preload(ld_preload);
    return preload.find("v4l2-compat") != std::string::npos ||
           preload.find("libcamerify") != std::string::npos ||
           preload.find("libcamera") != std::string::npos;
}

}  // namespace

CameraCapture::CameraCapture(int camera_index) {
    const bool libcamerify_active = isLibcamerifyActive();
    if (libcamerify_active) {
        std::cout << "[CameraCapture] libcamerify detected via LD_PRELOAD. "
                     "Prioritising default backend."
                  << std::endl;
    }

    bool opened = false;
    if (libcamerify_active) {
        opened = tryOpen(cap, "default backend index " + std::to_string(camera_index),
                         [&]() { return cap.open(camera_index); });
        if (!opened) {
            opened = tryOpen(cap, "V4L2 index " + std::to_string(camera_index), [&]() {
                return cap.open(camera_index, cv::CAP_V4L2);
            });
        }
    } else {
        // Default path when not wrapped by libcamerify.
        opened = tryOpen(cap, "V4L2 index " + std::to_string(camera_index), [&]() {
            return cap.open(camera_index, cv::CAP_V4L2);
        });
        if (!opened) {
            opened = tryOpen(cap, "default backend index " + std::to_string(camera_index),
                             [&]() { return cap.open(camera_index); });
        }
    }

    if (!opened && !libcamerify_active) {
        const std::vector<std::string> pipelines = buildLibcameraPipelines();
        for (std::size_t index = 0; index < pipelines.size() && !opened; ++index) {
            const std::string label =
                "GStreamer/libcamerasrc pipeline " + std::to_string(index + 1);
            opened = tryOpen(cap, label, [&]() {
                return cap.open(pipelines[index], cv::CAP_GSTREAMER);
            });
        }
    }

    if (!opened) {
        if (libcamerify_active) {
            std::cerr << "ERROR: Cannot open camera in libcamerify mode "
                         "(tried default backend and V4L2)."
                      << std::endl;
        } else {
            std::cerr << "ERROR: Cannot open camera. Tried V4L2, default backend, and libcamerasrc."
                      << std::endl;
        }
    }

    // Sets the internal buffer size to 1 to ensure the latest frame is fetched,
    // preventing the accumulation of delayed frames.
    if (cap.isOpened() && !cap.set(cv::CAP_PROP_BUFFERSIZE, 1)) {
        std::cerr << "[CameraCapture] Warning: CAP_PROP_BUFFERSIZE unsupported by backend."
                  << std::endl;
    }

    if (cap.isOpened()) {
        (void)cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        (void)cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        (void)cap.set(cv::CAP_PROP_FPS, 30);
        (void)cap.set(cv::CAP_PROP_READ_TIMEOUT_MSEC, 2000);
    }
}

CameraCapture::~CameraCapture() {
    if (cap.isOpened()) {
        cap.release();
    }
}

bool CameraCapture::waitForNextFrame(FrameEvent& output_event) {
    if (!cap.isOpened()) return false;

    // Tolerate occasional empty grabs from camera backends before declaring failure.
    constexpr int kMaxReadAttempts = 4;
    output_event.image_data.release();
    for (int attempt = 0; attempt < kMaxReadAttempts; ++attempt) {
        if (!cap.read(output_event.image_data)) {
            output_event.image_data.release();
        }

        if (!output_event.image_data.empty()) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Checks for valid frame data.
    if (output_event.image_data.empty()) {
        std::cerr << "WARNING: Empty frame captured (backend: ";
        if (cap.isOpened()) {
            std::cerr << cap.getBackendName();
        } else {
            std::cerr << "closed";
        }
        std::cerr << ")." << std::endl;
        return false;
    }

    // Captures the current system time immediately after the frame arrives.
    auto now = std::chrono::system_clock::now();
    output_event.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();

    return true;
}
