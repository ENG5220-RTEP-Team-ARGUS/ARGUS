#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

// Structure representing the output data: frame object and its timestamp.
struct FrameEvent {
    cv::Mat image_data;
    long long timestamp_ms;
};

class CameraCaptureBackend;

// Class for the camera capture module.
class CameraCapture {
public:
    enum class BackendPreference {
        Auto,
        OpenCvVideoCapture,
        Libcamera2OpenCv,
    };

    struct Options {
        int camera_index = 0;
        BackendPreference backend_preference = BackendPreference::Auto;
    };

    // Initializes the camera using the specified device index.
    explicit CameraCapture(int camera_index = 0);
    explicit CameraCapture(Options options);

    // Releases camera hardware resources.
    ~CameraCapture();

    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;
    CameraCapture(CameraCapture&&) = delete;
    CameraCapture& operator=(CameraCapture&&) = delete;

    // Blocks execution until a new frame is available.
    // Populates output_event with the frame and a timestamp.
    // Returns true on success, false on failure or empty frame.
    bool waitForNextFrame(FrameEvent& output_event);

    // Returns backend name if open, otherwise "CLOSED".
    std::string backendName() const;

    // Returns backend implementation family, for example OpenCV VideoCapture.
    std::string backendImplementation() const;

    BackendPreference backendPreference() const noexcept;

private:
    std::unique_ptr<CameraCaptureBackend> backend_;
    BackendPreference backend_preference_;
};
