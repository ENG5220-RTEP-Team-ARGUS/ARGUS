#include "CameraCapture.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

std::vector<std::string> buildLibcameraPipelines() {
    // Ordered from stricter caps to more permissive fallback.
    return {
        "libcamerasrc ! video/x-raw,format=NV12,width=640,height=480,framerate=30/1 "
        "! videoconvert ! video/x-raw,format=BGR "
        "! appsink drop=true max-buffers=1 sync=false",
        "libcamerasrc ! video/x-raw,width=640,height=480,framerate=30/1 "
        "! videoconvert ! video/x-raw,format=BGR "
        "! appsink drop=true max-buffers=1 sync=false",
        "libcamerasrc ! videoconvert ! video/x-raw,format=BGR "
        "! appsink drop=true max-buffers=1 sync=false"};
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

std::string buildV4l2DevicePath(int camera_index) {
    return "/dev/video" + std::to_string(camera_index);
}

const char* backendPreferenceToString(CameraCapture::BackendPreference preference) {
    switch (preference) {
        case CameraCapture::BackendPreference::Auto:
            return "auto";
        case CameraCapture::BackendPreference::OpenCvVideoCapture:
            return "opencv";
        case CameraCapture::BackendPreference::Libcamera2OpenCv:
            return "libcamera2opencv";
    }
    return "unknown";
}

}  // namespace

class CameraCaptureBackend {
public:
    virtual ~CameraCaptureBackend() = default;

    virtual bool isOpen() const noexcept = 0;
    virtual bool waitForNextFrame(FrameEvent& output_event) = 0;
    virtual std::string backendName() const = 0;
    virtual std::string backendImplementation() const = 0;
};

class OpenCvVideoCaptureBackend final : public CameraCaptureBackend {
public:
    explicit OpenCvVideoCaptureBackend(int camera_index) {
        const bool libcamerify_active = isLibcamerifyActive();
        if (libcamerify_active) {
            std::cout << "[CameraCapture] libcamerify detected via LD_PRELOAD. "
                         "Enforcing V4L2-first camera open policy."
                      << std::endl;
        }

        bool opened = false;
        if (libcamerify_active) {
            const std::string device_path = buildV4l2DevicePath(camera_index);
            opened = tryOpen(cap_, "V4L2 device path " + device_path, [&]() {
                return cap_.open(device_path, cv::CAP_V4L2);
            });
            if (!opened) {
                opened = tryOpen(cap_, "V4L2 index " + std::to_string(camera_index), [&]() {
                    return cap_.open(camera_index, cv::CAP_V4L2);
                });
            }
        } else {
            opened = tryOpen(cap_, "V4L2 index " + std::to_string(camera_index), [&]() {
                return cap_.open(camera_index, cv::CAP_V4L2);
            });
            if (!opened) {
                opened = tryOpen(cap_,
                                 "default backend index " + std::to_string(camera_index),
                                 [&]() { return cap_.open(camera_index); });
            }
        }

        if (!opened && !libcamerify_active) {
            const std::vector<std::string> pipelines = buildLibcameraPipelines();
            for (std::size_t index = 0; index < pipelines.size() && !opened; ++index) {
                const std::string label =
                    "GStreamer/libcamerasrc pipeline " + std::to_string(index + 1);
                opened = tryOpen(cap_, label, [&]() {
                    return cap_.open(pipelines[index], cv::CAP_GSTREAMER);
                });
            }
        }

        if (!opened) {
            if (libcamerify_active) {
                std::cerr << "ERROR: Cannot open camera in libcamerify mode "
                             "(tried /dev/videoN and V4L2 index only)."
                          << std::endl;
            } else {
                std::cerr
                    << "ERROR: Cannot open camera. Tried V4L2, default backend, and libcamerasrc."
                    << std::endl;
            }
        }

        if (cap_.isOpened() && !cap_.set(cv::CAP_PROP_BUFFERSIZE, 1)) {
            std::cerr
                << "[CameraCapture] Warning: CAP_PROP_BUFFERSIZE unsupported by backend."
                << std::endl;
        }

        if (cap_.isOpened()) {
            (void)cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            (void)cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            (void)cap_.set(cv::CAP_PROP_FPS, 30);
            (void)cap_.set(cv::CAP_PROP_READ_TIMEOUT_MSEC, 2000);
        }
    }

    ~OpenCvVideoCaptureBackend() override {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

    bool isOpen() const noexcept override {
        return cap_.isOpened();
    }

    bool waitForNextFrame(FrameEvent& output_event) override {
        if (!cap_.isOpened()) {
            return false;
        }

        constexpr int kMaxReadAttempts = 4;
        output_event.image_data.release();
        for (int attempt = 0; attempt < kMaxReadAttempts; ++attempt) {
            if (!cap_.read(output_event.image_data)) {
                output_event.image_data.release();
            }

            if (!output_event.image_data.empty()) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        if (output_event.image_data.empty()) {
            std::cerr << "WARNING: Empty frame captured (backend: ";
            if (cap_.isOpened()) {
                std::cerr << cap_.getBackendName();
            } else {
                std::cerr << "closed";
            }
            std::cerr << ")." << std::endl;
            return false;
        }

        const auto now = std::chrono::system_clock::now();
        output_event.timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        now.time_since_epoch())
                                        .count();
        return true;
    }

    std::string backendName() const override {
        if (!cap_.isOpened()) {
            return "CLOSED";
        }
        return cap_.getBackendName();
    }

    std::string backendImplementation() const override {
        return "OpenCV VideoCapture";
    }

private:
    cv::VideoCapture cap_;
};

std::unique_ptr<CameraCaptureBackend> makeCameraBackend(CameraCapture::Options options) {
    switch (options.backend_preference) {
        case CameraCapture::BackendPreference::Auto:
        case CameraCapture::BackendPreference::OpenCvVideoCapture: {
            auto backend = std::make_unique<OpenCvVideoCaptureBackend>(options.camera_index);
            if (backend->isOpen()) {
                return backend;
            }
            return nullptr;
        }
        case CameraCapture::BackendPreference::Libcamera2OpenCv:
            std::cerr << "[CameraCapture] requested backend '"
                      << backendPreferenceToString(options.backend_preference)
                      << "' is not integrated yet. "
                         "CameraCapture now has a backend boundary, but the active "
                         "implementation is still OpenCV VideoCapture."
                      << std::endl;
            return nullptr;
    }
    return nullptr;
}

CameraCapture::CameraCapture(int camera_index)
    : CameraCapture(Options{camera_index, BackendPreference::Auto}) {}

CameraCapture::CameraCapture(Options options)
    : backend_preference_(options.backend_preference) {
    backend_ = makeCameraBackend(options);
}

CameraCapture::~CameraCapture() = default;

bool CameraCapture::waitForNextFrame(FrameEvent& output_event) {
    if (!backend_) {
        return false;
    }
    return backend_->waitForNextFrame(output_event);
}

std::string CameraCapture::backendName() const {
    if (!backend_) {
        return "CLOSED";
    }
    return backend_->backendName();
}

std::string CameraCapture::backendImplementation() const {
    if (!backend_) {
        return "UNAVAILABLE";
    }
    return backend_->backendImplementation();
}

CameraCapture::BackendPreference CameraCapture::backendPreference() const noexcept {
    return backend_preference_;
}
