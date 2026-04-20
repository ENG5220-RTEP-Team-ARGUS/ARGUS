/**
 * @file CameraCapture.hpp
 * @brief Camera frame acquisition interface for ARGUS.
 */

#pragma once

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Single captured frame and its timing metadata.
 */
struct FrameEvent {
    cv::Mat image_data;  ///< Captured image.
    long long timestamp_ms;  ///< Epoch-style timestamp for log compatibility.
    std::chrono::steady_clock::time_point capture_timestamp;  ///< Monotonic capture time.
};

class CameraCaptureBackend;

/**
 * @brief Camera capture facade with pluggable backend implementations.
 */
class CameraCapture {
public:
    /**
     * @brief Backend selection policy.
     */
    enum class BackendPreference {
        Auto,  ///< Auto-select backend based on platform/runtime constraints.
        OpenCvVideoCapture,  ///< Force OpenCV VideoCapture backend.
        Libcamera2OpenCv,  ///< Force libcamera2opencv callback backend.
    };

    /**
     * @brief Construction options for camera capture.
     */
    struct Options {
        int camera_index = 0;  ///< Camera index passed to backend.
        BackendPreference backend_preference = BackendPreference::Auto;  ///< Requested backend policy.
        float focus_position = -1.0f;  ///< Focus request: -1.0 autofocus, 0.0 closest, 1.0 farthest.
    };

    /**
     * @brief Construct using explicit camera index.
     * @param camera_index Camera index.
     */
    explicit CameraCapture(int camera_index = 0);

    /**
     * @brief Construct using full options.
     * @param options Camera options.
     */
    explicit CameraCapture(Options options);

    /**
     * @brief Release camera hardware resources.
     */
    ~CameraCapture();

    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;
    CameraCapture(CameraCapture&&) = delete;
    CameraCapture& operator=(CameraCapture&&) = delete;

    /**
     * @brief Wait for the next captured frame.
     * @param output_event Output frame event populated on success.
     * @return `true` on successful frame delivery, otherwise `false`.
     */
    bool waitForNextFrame(FrameEvent& output_event);

    /**
     * @brief Return active backend name.
     * @return Backend name string, or `"CLOSED"` if backend is not active.
     */
    std::string backendName() const;

    /**
     * @brief Return active backend implementation family.
     * @return Implementation family (for example, OpenCV VideoCapture).
     */
    std::string backendImplementation() const;

    /**
     * @brief Return requested backend preference.
     * @return Backend preference enum.
     */
    BackendPreference backendPreference() const noexcept;

    /**
     * @brief Set camera focus position when backend/hardware supports it.
     * @param position Focus request: -1.0 autofocus, 0.0 closest, 1.0 farthest.
     * @return `true` when request is accepted by backend, otherwise `false`.
     */
    bool setFocusPosition(float position);

    /**
     * @brief Get current requested focus position.
     * @return Focus position in the same convention as setFocusPosition().
     */
    float getFocusPosition() const;

private:
    std::unique_ptr<CameraCaptureBackend> backend_;
    BackendPreference backend_preference_;
    float current_focus_position_ = -1.0f;
};
