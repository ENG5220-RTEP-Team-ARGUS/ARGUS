#include "CameraCapture.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(ARGUS_HAVE_LIBCAM2OPENCV)
#include <libcam2opencv.h>
#endif

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

CameraCapture::BackendPreference parseBackendPreference(const char* raw_value,
                                                        bool* recognised = nullptr) {
    if (recognised != nullptr) {
        *recognised = true;
    }

    if (raw_value == nullptr) {
        if (recognised != nullptr) {
            *recognised = false;
        }
        return CameraCapture::BackendPreference::Auto;
    }

    std::string value(raw_value);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (value == "auto") {
        return CameraCapture::BackendPreference::Auto;
    }
    if (value == "opencv" || value == "videocapture" || value == "opencvvideocapture") {
        return CameraCapture::BackendPreference::OpenCvVideoCapture;
    }
    if (value == "libcamera2opencv" || value == "libcamera" ||
        value == "cam2opencv") {
        return CameraCapture::BackendPreference::Libcamera2OpenCv;
    }

    if (recognised != nullptr) {
        *recognised = false;
    }
    return CameraCapture::BackendPreference::Auto;
}

CameraCapture::BackendPreference effectiveBackendPreference(
    CameraCapture::BackendPreference requested_preference) {
    if (requested_preference != CameraCapture::BackendPreference::Auto) {
        return requested_preference;
    }

    bool recognised = false;
    const auto env_preference =
        parseBackendPreference(std::getenv("ARGUS_CAMERA_BACKEND"), &recognised);
    if (recognised) {
        return env_preference;
    }

    return CameraCapture::BackendPreference::Auto;
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

#if defined(ARGUS_HAVE_LIBCAM2OPENCV)
class Libcamera2OpenCvBackend final : public CameraCaptureBackend {
public:
    explicit Libcamera2OpenCvBackend(int camera_index) {
        Libcam2OpenCVSettings settings;
        settings.cameraIndex = static_cast<unsigned int>(std::max(camera_index, 0));
        settings.width = 640;
        settings.height = 480;
        settings.framerate = 30;

        camera_.registerCallback([this](const cv::Mat& frame,
                                        const libcamera::ControlList&) {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_frame_ = frame.clone();
            const auto now = std::chrono::system_clock::now();
            latest_timestamp_ms_ =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch())
                    .count();
            ++frame_sequence_;
            condition_.notify_one();
        });

        try {
            camera_.start(settings);
            open_ = true;
            std::cout << "[CameraCapture] Opened via libcamera2opencv callback backend"
                      << std::endl;
        } catch (const std::exception& exception) {
            last_error_ = exception.what();
            std::cerr << "[CameraCapture] libcamera2opencv start failed: "
                      << last_error_ << std::endl;
            safeStop();
        } catch (...) {
            last_error_ = "unknown libcamera2opencv exception";
            std::cerr << "[CameraCapture] libcamera2opencv start failed: "
                      << last_error_ << std::endl;
            safeStop();
        }
    }

    ~Libcamera2OpenCvBackend() override {
        safeStop();
    }

    bool isOpen() const noexcept override {
        return open_;
    }

    bool waitForNextFrame(FrameEvent& output_event) override {
        if (!open_) {
            return false;
        }

        std::unique_lock<std::mutex> lock(mutex_);
        const bool ready = condition_.wait_for(
            lock,
            std::chrono::milliseconds(2000),
            [&]() { return frame_sequence_ > delivered_sequence_ || !open_; });

        if (!ready || (!open_ && frame_sequence_ <= delivered_sequence_)) {
            std::cerr << "WARNING: Empty frame captured (backend: libcamera2opencv)."
                      << std::endl;
            return false;
        }

        output_event.image_data = latest_frame_.clone();
        output_event.timestamp_ms = latest_timestamp_ms_;
        delivered_sequence_ = frame_sequence_;
        return !output_event.image_data.empty();
    }

    std::string backendName() const override {
        return open_ ? "LIBCAMERA2OPENCV" : "CLOSED";
    }

    std::string backendImplementation() const override {
        return "libcamera2opencv";
    }

private:
    void safeStop() noexcept {
        if (open_) {
            try {
                camera_.stop();
            } catch (...) {
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            open_ = false;
        }
        condition_.notify_all();
    }

    Libcam2OpenCV camera_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    cv::Mat latest_frame_;
    long long latest_timestamp_ms_{0};
    std::uint64_t frame_sequence_{0};
    std::uint64_t delivered_sequence_{0};
    bool open_{false};
    std::string last_error_;
};
#endif

std::unique_ptr<CameraCaptureBackend> makeCameraBackend(CameraCapture::Options options) {
    const auto effective_preference =
        effectiveBackendPreference(options.backend_preference);

    switch (effective_preference) {
        case CameraCapture::BackendPreference::Auto:
#if defined(ARGUS_HAVE_LIBCAM2OPENCV)
            {
                auto backend =
                    std::make_unique<Libcamera2OpenCvBackend>(options.camera_index);
                if (backend->isOpen()) {
                    return backend;
                }
                std::cerr << "[CameraCapture] libcamera2opencv backend unavailable; "
                             "falling back to OpenCV VideoCapture."
                          << std::endl;
            }
#endif
            [[fallthrough]];
        case CameraCapture::BackendPreference::OpenCvVideoCapture: {
            auto backend = std::make_unique<OpenCvVideoCaptureBackend>(options.camera_index);
            if (backend->isOpen()) {
                return backend;
            }
            return nullptr;
        }
        case CameraCapture::BackendPreference::Libcamera2OpenCv:
#if defined(ARGUS_HAVE_LIBCAM2OPENCV)
            {
                auto backend =
                    std::make_unique<Libcamera2OpenCvBackend>(options.camera_index);
                if (backend->isOpen()) {
                    return backend;
                }
                return nullptr;
            }
#else
            std::cerr << "[CameraCapture] requested backend '"
                      << backendPreferenceToString(effective_preference)
                      << "' is not available in this build. "
                         "Configure with -DARGUS_ENABLE_LIBCAMERA2OPENCV=ON "
                         "and install the required libcamera/turbojpeg dependencies."
                      << std::endl;
            return nullptr;
#endif
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
