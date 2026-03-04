#include "CameraCapture.hpp"
#include <chrono>
#include <iostream>

// =========================================================================
// Class Implementation
// =========================================================================

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

// =========================================================================
// Main Execution / Testing
// =========================================================================

// Stub representing the next module in the pipeline (Vision Safety Check).
void visionSafetyCheckModule(const FrameEvent& frame_event) {
    std::cout << "[Vision Safety Check] Received frame. Timestamp: "
        << frame_event.timestamp_ms << " ms | Resolution: "
        << frame_event.image_data.cols << "x" << frame_event.image_data.rows << std::endl;
}

int main() {
    CameraCapture camera_capture(0);
    FrameEvent current_frame_event;

    // Main event loop, driven by the hardware frame rate.
    while (true) {
        // Blocks until the next frame is ready.
        if (camera_capture.waitForNextFrame(current_frame_event)) {

            // Passes the output data to the downstream module.
            visionSafetyCheckModule(current_frame_event);

            // Displays the frame for debugging purposes.
            cv::imshow("Camera Source Preview", current_frame_event.image_data);

            // Exits the loop if 'q' is pressed.
            if (cv::waitKey(1) == 'q') {
                break;
            }
        }
        else {
            // Exits the loop if the camera fails or disconnects.
            break;
        }
    }

    // Cleans up GUI windows before exiting.
    cv::destroyAllWindows();
    return 0;
}