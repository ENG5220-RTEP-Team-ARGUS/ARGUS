/**
 * @file main.cpp
 * @brief CLI entry point for ARGUS runtime modes.
 */

#include "AppController.hpp"

#include <exception>
#include <iostream>
#include <string>

namespace {

void printUsage(const char* program_name) {
    std::cout
        << "Usage:\n"
        << "  " << program_name << "                Run Guardian FSM scenario demo\n"
        << "  " << program_name << " --button-test\n"
        << "  " << program_name << " --servo-calibrate\n"
        << "  " << program_name << " --servo-console\n"
        << "  " << program_name << " --motion-home\n"
        << "  " << program_name << " --motion-smoke-test [options]\n"
        << "  " << program_name << " --camera-backend-check [options]\n"
        << "  " << program_name << " --live-test [options]\n"
        << "\nOptions:\n"
        << "  --button-test             Run a GPIO button diagnostic loop\n"
        << "  --full-demo               Deprecated alias for --live-test\n"
        << "  --servo-calibrate         Run an interactive raw-pulse calibration console\n"
        << "  --servo-console           Run an interactive servo console (for example: 'base 90')\n"
        << "  --motion-home             Move all joints to logical 0 / home and exit\n"
        << "  --motion-smoke-test       Run the motion-only servo smoke test (all joints by default)\n"
        << "  --camera-backend-check    Capture a short sample and report backend/frame stats\n"
        << "  --base                    Smoke-test only the base servo\n"
        << "  --lower                   Smoke-test only the lower servo\n"
        << "  --upper                   Smoke-test only the upper servo\n"
        << "  --grip                    Smoke-test only the grip servo\n"
        << "\nOptions for --live-test:\n"
        << "  --camera-index <n>        Camera index (default: 0)\n"
        << "  --expected-marker-id <n>  Expected ArUco marker ID (default: 23)\n"
        << "  --camera-backend <name>   Camera backend: auto, libcamera2opencv, or opencv (default: auto)\n"
        << "  --auto-ack                Auto-send operator acknowledge when frozen\n"
        << "  --help                    Show this message\n"
        << "\nLive-test controls:\n"
        << "  space/button              Operator acknowledge\n"
        << "  0/1/2/3                   Select mode/routine\n"
        << "  d/a                       Base left/right\n"
        << "  w/s                       Forward/backward\n"
        << "  i/k                       Up/down\n"
        << "  l/j                       Open/close gripper\n"
        << "  +/-                       Adjust camera focus (Pi Camera Module 3 only, -=autofocus)\n"
        << "  esc                       Quit\n";
}

bool parseSmokeJointFlag(const std::string& arg, AppController::SmokeJoint& joint) {
    if (arg == "--base") {
        joint = AppController::SmokeJoint::Base;
        return true;
    }
    if (arg == "--lower") {
        joint = AppController::SmokeJoint::Lower;
        return true;
    }
    if (arg == "--upper") {
        joint = AppController::SmokeJoint::Upper;
        return true;
    }
    if (arg == "--grip") {
        joint = AppController::SmokeJoint::Grip;
        return true;
    }
    return false;
}

bool parseIntArg(const char* text, int& value) {
    try {
        const std::string raw(text);
        std::size_t parsed_count = 0;
        const int parsed = std::stoi(raw, &parsed_count);
        if (parsed_count != raw.size()) {
            return false;
        }
        value = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseCameraBackendArg(const std::string& raw,
                           CameraCapture::BackendPreference& preference) {
    if (raw == "auto") {
        preference = CameraCapture::BackendPreference::Auto;
        return true;
    }
    if (raw == "opencv" || raw == "videocapture") {
        preference = CameraCapture::BackendPreference::OpenCvVideoCapture;
        return true;
    }
    if (raw == "libcamera2opencv" || raw == "libcamera" || raw == "cam2opencv") {
        preference = CameraCapture::BackendPreference::Libcamera2OpenCv;
        return true;
    }
    return false;
}

}  // namespace

int main(int argc, char* argv[]) {
    bool run_button_test = false;
    bool run_live_test = false;
    bool run_servo_calibrate = false;
    bool run_servo_console = false;
    bool run_motion_home = false;
    bool run_motion_smoke_test = false;
    bool run_camera_backend_check = false;
    AppController::LiveTestOptions options;
    AppController::MotionSmokeTestOptions smoke_options;
    bool smoke_joint_selected = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);

        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }

        if (arg == "--live-test") {
            run_live_test = true;
            continue;
        }

        if (arg == "--button-test") {
            run_button_test = true;
            continue;
        }

        if (arg == "--full-demo") {
            std::cerr << "[CLI] --full-demo is deprecated; using --live-test."
                      << std::endl;
            run_live_test = true;
            continue;
        }

        if (arg == "--servo-calibrate") {
            run_servo_calibrate = true;
            continue;
        }

        if (arg == "--servo-console") {
            run_servo_console = true;
            continue;
        }

        if (arg == "--motion-home") {
            run_motion_home = true;
            continue;
        }

        if (arg == "--motion-smoke-test") {
            run_motion_smoke_test = true;
            continue;
        }

        if (arg == "--camera-backend-check") {
            run_camera_backend_check = true;
            continue;
        }

        if (parseSmokeJointFlag(arg, smoke_options.joint)) {
            if (smoke_joint_selected) {
                std::cerr << "Choose one smoke-test joint flag at a time." << std::endl;
                printUsage(argv[0]);
                return 1;
            }
            run_motion_smoke_test = true;
            smoke_joint_selected = true;
            continue;
        }

        if (arg == "--auto-ack") {
            options.auto_ack = true;
            continue;
        }

        if ((arg == "--camera-index" || arg == "--expected-marker-id" ||
             arg == "--camera-backend") &&
            i + 1 < argc) {
            if (arg == "--camera-backend") {
                if (!parseCameraBackendArg(argv[i + 1], options.backend_preference)) {
                    std::cerr << "Invalid camera backend for " << arg << ": "
                              << argv[i + 1] << std::endl;
                    return 1;
                }

                ++i;
                continue;
            }

            int parsed_value = 0;
            if (!parseIntArg(argv[i + 1], parsed_value)) {
                std::cerr << "Invalid numeric value for " << arg << ": "
                          << argv[i + 1] << std::endl;
                return 1;
            }

            if (arg == "--camera-index") {
                options.camera_index = parsed_value;
            } else {
                options.expected_marker_id = parsed_value;
            }

            ++i;
            continue;
        }

        std::cerr << "Unknown or incomplete argument: " << arg << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    if ((run_button_test && run_live_test) ||
        (run_button_test && run_servo_calibrate) ||
        (run_button_test && run_servo_console) ||
        (run_button_test && run_motion_home) ||
        (run_button_test && run_motion_smoke_test) ||
        (run_button_test && run_camera_backend_check) ||
        (run_live_test && run_servo_calibrate) ||
        (run_live_test && run_servo_console) ||
        (run_live_test && run_motion_home) ||
        (run_live_test && run_motion_smoke_test) ||
        (run_live_test && run_camera_backend_check) ||
        (run_servo_calibrate && run_servo_console) ||
        (run_servo_calibrate && run_motion_home) ||
        (run_servo_calibrate && run_motion_smoke_test) ||
        (run_servo_calibrate && run_camera_backend_check) ||
        (run_servo_console && run_motion_home) ||
        (run_servo_console && run_motion_smoke_test) ||
        (run_servo_console && run_camera_backend_check) ||
        (run_motion_home && run_motion_smoke_test) ||
        (run_motion_home && run_camera_backend_check) ||
        (run_motion_smoke_test && run_camera_backend_check)) {
        std::cerr << "Choose exactly one mode: --button-test, --live-test, --servo-calibrate, --servo-console, --motion-home, --motion-smoke-test, or --camera-backend-check."
                  << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    AppController app_controller;
    if (run_button_test) {
        return app_controller.runButtonTest();
    }

    if (run_servo_calibrate) {
        return app_controller.runServoCalibration();
    }

    if (run_servo_console) {
        return app_controller.runInteractiveServoConsole();
    }

    if (run_motion_home) {
        return app_controller.runMotionHomePose();
    }

    if (run_motion_smoke_test) {
        return app_controller.runMotionSmokeTest(smoke_options);
    }

    if (run_camera_backend_check) {
        return app_controller.runCameraBackendCheck(options);
    }

    if (run_live_test) {
        return app_controller.runLiveMarkerTest(options);
    }

    return app_controller.runGuardianScenarioDemo();
}
