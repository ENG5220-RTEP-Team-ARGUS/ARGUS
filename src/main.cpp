#include "CameraCapture.hpp"
#include "GuardianStateMachine.hpp"
#include "RobotInterlock.hpp"
#include "VisionProcessor.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace {

struct LiveTestOptions {
    int camera_index = 0;
    int expected_marker_id = 23;
    bool auto_ack = false;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage:\n"
        << "  " << program_name << "                Run Guardian FSM scenario demo\n"
        << "  " << program_name << " --live-test [options]\n"
        << "\nOptions for --live-test:\n"
        << "  --camera-index <n>        Camera index (default: 0)\n"
        << "  --expected-marker-id <n>  Expected ArUco marker ID (default: 23)\n"
        << "  --auto-ack                Auto-send operator acknowledge when frozen\n"
        << "  --help                    Show this message\n";
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

double computeFocusScore(const cv::Mat& image) {
    if (image.empty()) {
        return 0.0;
    }

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    return stddev[0] * stddev[0];
}

const char* focusQualityLabel(double focus_score) {
    // Heuristic bands for quick focus debugging.
    if (focus_score < 60.0) {
        return "BLURRY";
    }
    if (focus_score < 180.0) {
        return "SOFT";
    }
    return "SHARP";
}

std::string formatFocusScore(double focus_score) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << focus_score;
    return oss.str();
}

const char* safetyStateToString(SafetyState state) {
    switch (state) {
        case SafetyState::SAFE:
            return "SAFE";
        case SafetyState::TOOL_NOT_DETECTED:
            return "TOOL_NOT_DETECTED";
        case SafetyState::OUTSIDE_ALLOWED_ZONE:
            return "OUTSIDE_ALLOWED_ZONE";
        case SafetyState::EXCESSIVE_SPEED:
            return "EXCESSIVE_SPEED";
        case SafetyState::INVALID_ORIENTATION:
            return "INVALID_ORIENTATION";
        default:
            return "UNKNOWN";
    }
}

FreezeReason mapSafetyToFreezeReason(SafetyState state) {
    switch (state) {
        case SafetyState::TOOL_NOT_DETECTED:
            return FreezeReason::MARKER_LOST;
        case SafetyState::OUTSIDE_ALLOWED_ZONE:
            return FreezeReason::MARKER_OUT_OF_ROI;
        case SafetyState::EXCESSIVE_SPEED:
        case SafetyState::INVALID_ORIENTATION:
            return FreezeReason::POSITION_ERROR;
        case SafetyState::SAFE:
        default:
            return FreezeReason::UNKNOWN_FAULT;
    }
}

const char* freezeReasonToString(FreezeReason reason) {
    switch (reason) {
        case FreezeReason::NONE:
            return "NONE";
        case FreezeReason::MARKER_LOST:
            return "MARKER_LOST";
        case FreezeReason::MARKER_OUT_OF_ROI:
            return "MARKER_OUT_OF_ROI";
        case FreezeReason::VISION_TIMEOUT:
            return "VISION_TIMEOUT";
        case FreezeReason::POSITION_ERROR:
            return "POSITION_ERROR";
        case FreezeReason::WATCHDOG_TIMEOUT:
            return "WATCHDOG_TIMEOUT";
        case FreezeReason::UNKNOWN_FAULT:
        default:
            return "UNKNOWN_FAULT";
    }
}

class LoggingRobotHardware final : public RobotHardware {
public:
    void freezeMotion() noexcept override {
        std::cout << "[INTERLOCK_HW] freezeMotion()" << std::endl;
    }

    void enableMotion() noexcept override {
        std::cout << "[INTERLOCK_HW] enableMotion()" << std::endl;
    }
};

void robotArmFreezeHandler() {
    std::cout << ">>> ROBOTIC ARM: Emergency stop activated! <<<" << std::endl;
}

void robotArmClearFreezeHandler() {
    std::cout << ">>> ROBOTIC ARM: Motion resumed, system operational <<<"
              << std::endl;
}

void stateChangeHandler(GuardianState from, GuardianState to) {
    (void)from;
    (void)to;
    std::cout << ">>> STATE CHANGE NOTIFICATION: System transitioned <<<"
              << std::endl;
}

int runGuardianScenarioDemo() {
    GuardianStateMachine guardian(2, 3);
    guardian.setOnFreezeCallback(robotArmFreezeHandler);
    guardian.setOnClearFreezeCallback(robotArmClearFreezeHandler);
    guardian.setOnStateChangeCallback(stateChangeHandler);

    std::cout << "\n========== GUARDIAN STATE MACHINE TEST ==========\n" << std::endl;

    std::cout << "Scenario 1: Normal Operation" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "Scenario 2: Single Bad Frame" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "Scenario 3: freezeCount Consecutive Bad Frames (Freeze)"
              << std::endl;
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.printStatus();

    std::cout << "Scenario 4: Frames During Frozen State" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "Scenario 5: Operator Acknowledgment/Reset" << std::endl;
    guardian.operatorAcknowledge();
    guardian.printStatus();

    std::cout << "Scenario 6: Bad Frame During Reset" << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_BAD);
    guardian.printStatus();

    std::cout << "Scenario 7: recoverCount Consecutive Good Frames (Clear Freeze)"
              << std::endl;
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.processFrame(FrameStatus::FRAME_GOOD);
    guardian.printStatus();

    std::cout << "========== TEST COMPLETE ==========\n" << std::endl;
    return 0;
}

int runLiveMarkerTest(const LiveTestOptions& options) {
    constexpr const char* kWindowName = "ARGUS Live Test";
    constexpr int kLiveFreezeBadFrameThreshold = 30;  // ~1 second at ~30 FPS
    constexpr int kLiveRecoverGoodFrameThreshold = 3;

    std::cout
        << "[LIVE_TEST] Starting live marker safety test mode\n"
        << "[LIVE_TEST] Camera index: " << options.camera_index << "\n"
        << "[LIVE_TEST] Expected marker ID: " << options.expected_marker_id
        << "\n"
        << "[LIVE_TEST] Auto operator ack: "
        << (options.auto_ack ? "ON" : "OFF") << "\n"
        << "[LIVE_TEST] Controls: a=arm, d=disarm, q=quit\n"
        << "[LIVE_TEST] Starting in DISARMED setup mode\n"
        << "[LIVE_TEST] Guardian thresholds: freeze after "
        << kLiveFreezeBadFrameThreshold
        << " consecutive bad frames, recover after "
        << kLiveRecoverGoodFrameThreshold
        << " consecutive good frames.\n"
        << "[LIVE_TEST] Focus debug enabled: FOCUS_SCORE (Laplacian variance), "
           "higher usually means sharper marker edges.\n";

    CameraCapture camera_capture(options.camera_index);
    cv::namedWindow(kWindowName, cv::WINDOW_AUTOSIZE);

    VisionConfig vision_config;
    vision_config.expectedMarkerId = options.expected_marker_id;
    VisionProcessor vision_processor(vision_config);

    LoggingRobotHardware logging_hardware;
    std::unique_ptr<GuardianStateMachine> guardian;
    std::unique_ptr<RobotInterlock> interlock;
    bool guardian_armed = false;

    FreezeReason pending_reason = FreezeReason::UNKNOWN_FAULT;

    auto resetEnforcementState = [&]() {
        guardian = std::make_unique<GuardianStateMachine>(
            kLiveFreezeBadFrameThreshold,
            kLiveRecoverGoodFrameThreshold);
        interlock = std::make_unique<RobotInterlock>(logging_hardware);

        guardian->setOnFreezeCallback([&]() {
            interlock->onControlEvent(ControlEvent::FREEZE_NOW, pending_reason);
        });

        guardian->setOnClearFreezeCallback([&]() {
            interlock->onControlEvent(ControlEvent::ALLOW_MOTION);
        });

        guardian->setOnStateChangeCallback([&](GuardianState from, GuardianState to) {
            std::cout << "[GUARDIAN] " << guardian->stateToString(from) << " -> "
                      << guardian->stateToString(to) << std::endl;
        });
    };

    resetEnforcementState();

    FrameEvent frame_event;
    std::uint64_t frame_index = 0;
    bool processed_any_frame = false;
    int consecutive_capture_failures = 0;

    while (true) {
        if (!camera_capture.waitForNextFrame(frame_event)) {
            ++consecutive_capture_failures;
            std::cerr << "[LIVE_TEST] Frame capture failed ("
                      << consecutive_capture_failures
                      << "/30). Retrying..." << std::endl;

            if (consecutive_capture_failures >= 30) {
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        consecutive_capture_failures = 0;
        processed_any_frame = true;

        const SafetyResult result =
            vision_processor.process(frame_event.image_data,
                                     std::chrono::steady_clock::now());
        const double focus_score = computeFocusScore(frame_event.image_data);
        const char* focus_quality = focusQualityLabel(focus_score);

        const bool frame_is_safe = (result.state == SafetyState::SAFE);
        if (!frame_is_safe) {
            pending_reason = mapSafetyToFreezeReason(result.state);
        }

        std::string guardian_state_text = "DISARMED_SETUP";
        std::string interlock_state_text = "DISARMED";
        std::string freeze_reason_text = "N/A";

        if (guardian_armed) {
            guardian->processFrame(frame_is_safe ? FrameStatus::FRAME_GOOD
                                                 : FrameStatus::FRAME_BAD);

            if (options.auto_ack &&
                guardian->getState() == GuardianState::FROZEN_UNSAFE) {
                guardian->operatorAcknowledge();
                interlock->operatorAcknowledge();
                std::cout << "[LIVE_TEST] Operator acknowledge sent automatically"
                          << std::endl;
            }

            interlock->guardianHeartbeat(static_cast<std::uint32_t>(frame_index));

            guardian_state_text = guardian->getCurrentStateString();
            interlock_state_text =
                interlock->motionAllowed() ? "MOTION_ALLOWED" : "FROZEN";
            freeze_reason_text = freezeReasonToString(interlock->freezeReason());
        }

        std::cout << "[LIVE_TEST] frame=" << frame_index
                  << " armed=" << (guardian_armed ? "YES" : "NO")
                  << " can_arm=" << (frame_is_safe ? "YES" : "NO")
                  << " vision=" << safetyStateToString(result.state)
                  << " focus_score=" << formatFocusScore(focus_score)
                  << " focus=" << focus_quality
                  << " guardian=" << guardian_state_text
                  << " interlock=" << interlock_state_text
                  << " freeze_reason=" << freeze_reason_text
                  << " processing_us=" << result.processing_time.count()
                  << std::endl;

        cv::Mat display_frame = frame_event.image_data.clone();
        const bool decision_is_safe = guardian_armed
                                          ? ((result.state == SafetyState::SAFE) &&
                                             (guardian->getState() ==
                                              GuardianState::SAFE_MONITORING) &&
                                             interlock->motionAllowed())
                                          : (result.state == SafetyState::SAFE);
        const cv::Scalar decision_color =
            decision_is_safe ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

        cv::putText(display_frame,
                    std::string("VISION: ") + safetyStateToString(result.state),
                    cv::Point(16, 30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 255, 255),
                    2);

        cv::putText(display_frame,
                    std::string("GUARDIAN: ") + guardian_state_text,
                    cv::Point(16, 60),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 255, 255),
                    2);

        cv::putText(display_frame,
                    std::string("INTERLOCK: ") + interlock_state_text,
                    cv::Point(16, 90),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 255, 255),
                    2);

        cv::putText(display_frame,
                    std::string("CONTROL: ") +
                        (guardian_armed ? "ARMED (d=disarm)" : "DISARMED (a=arm)") +
                        " | CAN_ARM: " + (frame_is_safe ? "YES" : "NO"),
                    cv::Point(16, 120),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    cv::Scalar(255, 255, 0),
                    2);

        cv::putText(display_frame,
                    guardian_armed ? (decision_is_safe ? "SAFE" : "UNSAFE")
                                   : (frame_is_safe ? "OBSERVED_SAFE"
                                                    : "OBSERVED_UNSAFE"),
                    cv::Point(16, 155),
                    cv::FONT_HERSHEY_DUPLEX,
                    1.0,
                    decision_color,
                    2);

        cv::putText(display_frame,
                    "Expected marker ID: " + std::to_string(options.expected_marker_id),
                    cv::Point(16, 185),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    cv::Scalar(180, 180, 180),
                    1);

        const cv::Scalar focus_color =
            (focus_score < 60.0) ? cv::Scalar(0, 0, 255)
                                 : ((focus_score < 180.0) ? cv::Scalar(0, 255, 255)
                                                           : cv::Scalar(0, 255, 0));
        cv::putText(display_frame,
                    "FOCUS_SCORE: " + formatFocusScore(focus_score) + " (" +
                        std::string(focus_quality) + ")",
                    cv::Point(16, 215),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    focus_color,
                    2);

        cv::putText(display_frame,
                    "Focus hint: adjust lens/ring until score rises and marker is stable",
                    cv::Point(16, 240),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(200, 200, 200),
                    1);

        cv::imshow(kWindowName, display_frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            std::cout << "[LIVE_TEST] Exit requested from display window (q)." << std::endl;
            break;
        }

        if (key == 'a' || key == 'A') {
            if (guardian_armed) {
                std::cout << "[LIVE_TEST] ARM request ignored: guardian already armed."
                          << std::endl;
            } else if (!frame_is_safe) {
                std::cout << "[LIVE_TEST] ARM request rejected: current observed condition is "
                             "unsafe (vision="
                          << safetyStateToString(result.state) << ")." << std::endl;
            } else {
                resetEnforcementState();
                guardian_armed = true;
                std::cout << "[LIVE_TEST] ARM accepted: guardian enforcement is now ACTIVE."
                          << std::endl;
            }
        }

        if (key == 'd' || key == 'D') {
            if (!guardian_armed) {
                std::cout << "[LIVE_TEST] DISARM request ignored: guardian already disarmed."
                          << std::endl;
            } else {
                guardian_armed = false;
                resetEnforcementState();
                std::cout << "[LIVE_TEST] DISARM accepted: guardian enforcement is now INACTIVE "
                             "(setup/observation mode)."
                          << std::endl;
            }
        }

        ++frame_index;
    }

    cv::destroyWindow(kWindowName);

    if (!processed_any_frame) {
        std::cerr << "[LIVE_TEST] No frames processed. Check camera availability."
                  << std::endl;
        return 1;
    }

    std::cerr << "[LIVE_TEST] Frame stream ended." << std::endl;
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    bool run_live_test = false;
    LiveTestOptions options;

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

        if (arg == "--auto-ack") {
            options.auto_ack = true;
            continue;
        }

        if ((arg == "--camera-index" || arg == "--expected-marker-id") &&
            i + 1 < argc) {
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

    if (run_live_test) {
        return runLiveMarkerTest(options);
    }

    return runGuardianScenarioDemo();
}
