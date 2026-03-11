#include "CameraCapture.hpp"
#include "GuardianStateMachine.hpp"
#include "RobotInterlock.hpp"
#include "VisionProcessor.hpp"

#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string>

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
    std::cout
        << "[LIVE_TEST] Starting live marker safety test mode\n"
        << "[LIVE_TEST] Camera index: " << options.camera_index << "\n"
        << "[LIVE_TEST] Expected marker ID: " << options.expected_marker_id
        << "\n"
        << "[LIVE_TEST] Auto operator ack: "
        << (options.auto_ack ? "ON" : "OFF") << "\n"
        << "[LIVE_TEST] Stop with Ctrl+C\n";

    CameraCapture camera_capture(options.camera_index);

    VisionConfig vision_config;
    vision_config.expectedMarkerId = options.expected_marker_id;
    VisionProcessor vision_processor(vision_config);

    GuardianStateMachine guardian(2, 3);
    LoggingRobotHardware logging_hardware;
    RobotInterlock interlock(logging_hardware);

    FreezeReason pending_reason = FreezeReason::UNKNOWN_FAULT;

    guardian.setOnFreezeCallback([&]() {
        interlock.onControlEvent(ControlEvent::FREEZE_NOW, pending_reason);
    });

    guardian.setOnClearFreezeCallback([&]() {
        interlock.onControlEvent(ControlEvent::ALLOW_MOTION);
    });

    guardian.setOnStateChangeCallback([&](GuardianState from, GuardianState to) {
        std::cout << "[GUARDIAN] " << guardian.stateToString(from) << " -> "
                  << guardian.stateToString(to) << std::endl;
    });

    FrameEvent frame_event;
    std::uint64_t frame_index = 0;
    bool processed_any_frame = false;

    while (camera_capture.waitForNextFrame(frame_event)) {
        processed_any_frame = true;

        const SafetyResult result =
            vision_processor.process(frame_event.image_data,
                                     std::chrono::steady_clock::now());

        const bool frame_is_safe = (result.state == SafetyState::SAFE);
        if (!frame_is_safe) {
            pending_reason = mapSafetyToFreezeReason(result.state);
        }

        guardian.processFrame(frame_is_safe ? FrameStatus::FRAME_GOOD
                                            : FrameStatus::FRAME_BAD);

        if (options.auto_ack &&
            guardian.getState() == GuardianState::FROZEN_UNSAFE) {
            guardian.operatorAcknowledge();
            interlock.operatorAcknowledge();
            std::cout << "[LIVE_TEST] Operator acknowledge sent automatically"
                      << std::endl;
        }

        interlock.guardianHeartbeat(static_cast<std::uint32_t>(frame_index));

        std::cout << "[LIVE_TEST] frame=" << frame_index
                  << " vision=" << safetyStateToString(result.state)
                  << " guardian=" << guardian.getCurrentStateString()
                  << " interlock="
                  << (interlock.motionAllowed() ? "MOTION_ALLOWED" : "FROZEN")
                  << " freeze_reason="
                  << freezeReasonToString(interlock.freezeReason())
                  << " processing_us=" << result.processing_time.count()
                  << std::endl;

        ++frame_index;
    }

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
