#include "AppController.hpp"

#include "CameraCapture.hpp"
#include "GuardianStateMachine.hpp"
#include "PhysicalButtonModule.hpp"
#include "RobotInterlock.hpp"
#include "VisionProcessor.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

namespace {

constexpr const char* kLiveWindowName = "ARGUS Live Test";
constexpr const char* kDefaultI2cDevicePath = "/dev/i2c-1";
constexpr std::uint8_t kDefaultPca9685Address = 0x40;
constexpr float kDefaultPwmFrequencyHz = 50.0f;
constexpr std::uint8_t kBaseServoChannel = 0;
constexpr std::uint8_t kLowerServoChannel = 4;
constexpr std::uint8_t kUpperServoChannel = 8;
constexpr std::uint8_t kGripServoChannel = 12;
constexpr int kLiveFreezeBadFrameThreshold = 30;
constexpr int kLiveRecoverGoodFrameThreshold = 3;
constexpr std::chrono::milliseconds kSmokeStepDwell{3000};
constexpr std::uint16_t kSmokeNeutralPulseTicks = 300;
constexpr int kSmokeBaseMinOffset = -90;
constexpr int kSmokeBaseMaxOffset = 90;
constexpr int kSmokeLowerMinOffset = -90;
constexpr int kSmokeLowerMaxOffset = 90;
constexpr int kSmokeUpperMinOffset = -90;
constexpr int kSmokeUpperMaxOffset = 90;
constexpr int kSmokeGripMinOffset = -90;
constexpr int kSmokeGripMaxOffset = 90;
constexpr int kSmokePositiveStep = 90;
constexpr int kSmokeNegativeStep = -90;

class MotionControllerHardwareAdapter final : public RobotHardware {
public:
    explicit MotionControllerHardwareAdapter(MotionController& motion_controller) noexcept
        : motion_controller_(motion_controller) {}

    bool freezeMotion() noexcept override {
        motion_controller_.freeze();
        if (motion_controller_.outputState() == MotionOutputState::FAULT) {
            std::cerr << "[MOTION] freezeMotion() failed: "
                      << motion_controller_.lastErrorString() << std::endl;
            return false;
        }
        return true;
    }

    bool enableMotion() noexcept override {
        if (!motion_controller_.enable()) {
            std::cerr << "[MOTION] enableMotion() failed: "
                      << motion_controller_.lastErrorString() << std::endl;
            return false;
        }
        return true;
    }

private:
    MotionController& motion_controller_;
};

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

const char* interlockStateToString(InterlockState state) {
    switch (state) {
        case InterlockState::SAFE:
            return "SAFE";
        case InterlockState::FROZEN:
            return "FROZEN";
        case InterlockState::FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

const char* motionControllerStateToString(MotionOutputState state) {
    switch (state) {
        case MotionOutputState::UNINITIALISED:
            return "UNINITIALISED";
        case MotionOutputState::DISABLED:
            return "DISABLED";
        case MotionOutputState::ENABLED:
            return "ENABLED";
        case MotionOutputState::FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

struct SmokeJointOffsets {
    int base{0};
    int lower{0};
    int upper{0};
    int grip{0};
};

struct SmokeJointSpec {
    const char* logical_name;
    const char* mearm_name;
    const char* visual_check;
    std::uint8_t channel;
    int min_offset;
    int max_offset;
};

struct SmokeJointRunPlan {
    std::array<std::size_t, MotionController::kServoCount> indices{};
    std::size_t count{0};
};

constexpr std::array<SmokeJointSpec, MotionController::kServoCount> kSmokeJointSpecs = {{
    {"base", "MeArm BASE", "yaw left/right", kBaseServoChannel, kSmokeBaseMinOffset, kSmokeBaseMaxOffset},
    {"lower", "MeArm LEFT", "raise/lower", kLowerServoChannel, kSmokeLowerMinOffset, kSmokeLowerMaxOffset},
    {"upper", "MeArm RIGHT", "bend/extend", kUpperServoChannel, kSmokeUpperMinOffset, kSmokeUpperMaxOffset},
    {"grip", "MeArm CLAW", "open/close", kGripServoChannel, kSmokeGripMinOffset, kSmokeGripMaxOffset},
}};

constexpr SmokeJointOffsets kSmokeHomePose{0, 0, 0, 0};

const char* smokeJointSelectionToString(AppController::SmokeJoint joint) {
    switch (joint) {
        case AppController::SmokeJoint::All:
            return "all";
        case AppController::SmokeJoint::Base:
            return "base";
        case AppController::SmokeJoint::Lower:
            return "lower";
        case AppController::SmokeJoint::Upper:
            return "upper";
        case AppController::SmokeJoint::Grip:
            return "grip";
        default:
            return "unknown";
    }
}

SmokeJointRunPlan makeSmokeJointRunPlan(AppController::SmokeJoint joint) {
    SmokeJointRunPlan plan{};

    switch (joint) {
        case AppController::SmokeJoint::Base:
            plan.indices[0] = 0;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::Lower:
            plan.indices[0] = 1;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::Upper:
            plan.indices[0] = 2;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::Grip:
            plan.indices[0] = 3;
            plan.count = 1;
            break;
        case AppController::SmokeJoint::All:
        default:
            plan.indices[0] = 0;
            plan.indices[1] = 1;
            plan.indices[2] = 2;
            plan.indices[3] = 3;
            plan.count = 4;
            break;
    }

    return plan;
}

int clampOffsetValue(int value, int min_value, int max_value, bool& clamped) {
    const int bounded = std::clamp(value, min_value, max_value);
    if (bounded != value) {
        clamped = true;
    }
    return bounded;
}

SmokeJointOffsets clampSmokeOffsets(const SmokeJointOffsets& requested,
                                    bool& clamped) {
    SmokeJointOffsets bounded = requested;
    bounded.base = clampOffsetValue(requested.base,
                                    kSmokeBaseMinOffset,
                                    kSmokeBaseMaxOffset,
                                    clamped);
    bounded.lower = clampOffsetValue(requested.lower,
                                     kSmokeLowerMinOffset,
                                     kSmokeLowerMaxOffset,
                                     clamped);
    bounded.upper = clampOffsetValue(requested.upper,
                                     kSmokeUpperMinOffset,
                                     kSmokeUpperMaxOffset,
                                     clamped);
    bounded.grip = clampOffsetValue(requested.grip,
                                    kSmokeGripMinOffset,
                                    kSmokeGripMaxOffset,
                                    clamped);
    return bounded;
}

std::uint16_t offsetToPulseTicks(int offset, bool& clamped) {
    const int raw = static_cast<int>(kSmokeNeutralPulseTicks) + offset;
    const int bounded = std::clamp(
        raw, 0, static_cast<int>(MotionController::kMaxPulseTicks));
    if (bounded != raw) {
        clamped = true;
    }
    return static_cast<std::uint16_t>(bounded);
}

MeArmJointTargets makeSmokeTargets(const SmokeJointOffsets& requested,
                                   bool& clamped) {
    const SmokeJointOffsets bounded = clampSmokeOffsets(requested, clamped);
    return {
        offsetToPulseTicks(bounded.base, clamped),
        offsetToPulseTicks(bounded.lower, clamped),
        offsetToPulseTicks(bounded.upper, clamped),
        offsetToPulseTicks(bounded.grip, clamped)};
}

std::string formatOffsets(const SmokeJointOffsets& offsets) {
    std::ostringstream oss;
    oss << "{base=" << offsets.base
        << ", lower=" << offsets.lower
        << ", upper=" << offsets.upper
        << ", grip=" << offsets.grip
        << "}";
    return oss.str();
}

std::string formatTargets(const MeArmJointTargets& targets) {
    std::ostringstream oss;
    oss << "{base=" << targets.base_ticks
        << ", lower=" << targets.lower_ticks
        << ", upper=" << targets.upper_ticks
        << ", gripper=" << targets.gripper_ticks
        << "}";
    return oss.str();
}

}  // namespace

AppController::AppController() noexcept = default;

AppController::~AppController() noexcept = default;

int AppController::runGuardianScenarioDemo() {
    GuardianStateMachine guardian(2, 3);
    guardian.setOnFreezeCallback([]() {
        std::cout << ">>> ROBOTIC ARM: Emergency stop activated! <<<" << std::endl;
    });
    guardian.setOnClearFreezeCallback([]() {
        std::cout << ">>> ROBOTIC ARM: Motion resumed, system operational <<<"
                  << std::endl;
    });
    guardian.setOnStateChangeCallback([](GuardianState, GuardianState) {
        std::cout << ">>> STATE CHANGE NOTIFICATION: System transitioned <<<"
                  << std::endl;
    });

    std::cout << "\n========== GUARDIAN STATE MACHINE TEST ==========\n"
              << std::endl;

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

int AppController::runMotionSmokeTest(const MotionSmokeTestOptions& options) {
    std::cout
        << "[SMOKE] joint=" << smokeJointSelectionToString(options.joint)
        << " 0 -> -90 -> +90 -> 0"
        << "  wait=3s\n";

    MotionChannelMap channel_map{};
    channel_map.base = kBaseServoChannel;
    channel_map.lower = kLowerServoChannel;
    channel_map.upper = kUpperServoChannel;
    channel_map.gripper = kGripServoChannel;

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       channel_map)) {
        std::cerr << "[SMOKE] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[SMOKE] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    auto makeJointOffsets = [](std::size_t joint_index, int joint_offset) {
        SmokeJointOffsets offsets{};
        switch (joint_index) {
            case 0:
                offsets.base = joint_offset;
                break;
            case 1:
                offsets.lower = joint_offset;
                break;
            case 2:
                offsets.upper = joint_offset;
                break;
            case 3:
                offsets.grip = joint_offset;
                break;
            default:
                break;
        }
        return offsets;
    };

    auto stagePose = [&](const std::string& label,
                         const SmokeJointOffsets& requested) {
        bool clamped = false;
        const SmokeJointOffsets bounded = clampSmokeOffsets(requested, clamped);
        const MeArmJointTargets targets = makeSmokeTargets(bounded, clamped);

        if (!motion_controller_.setTargets(targets)) {
            return fail(std::string("failed to stage pose ") + label + ": " +
                        motion_controller_.lastErrorString());
        }

        std::cout << "[SMOKE] " << label;
        if (clamped) {
            std::cout << " [clamped]";
        }
        std::cout << std::endl;
        std::cout << "[SMOKE] wait 3s" << std::endl;
        std::this_thread::sleep_for(kSmokeStepDwell);
        return 0;
    };

    bool home_clamped = false;
    const MeArmJointTargets home_targets =
        makeSmokeTargets(kSmokeHomePose, home_clamped);

    if (!motion_controller_.setTargets(home_targets)) {
        return fail(std::string("failed to stage home pose: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::cout << "[SMOKE] all -> 0" << std::endl;
    std::cout << "[SMOKE] wait 3s" << std::endl;
    std::this_thread::sleep_for(kSmokeStepDwell);

    const SmokeJointRunPlan plan = makeSmokeJointRunPlan(options.joint);
    if (plan.count == 0) {
        return fail("no smoke-test joint selected");
    }

    auto runJointSweep = [&](std::size_t index) {
        const SmokeJointSpec& spec = kSmokeJointSpecs[index];
        std::cout << "[SMOKE] " << spec.logical_name << std::endl;

        const std::string home_label = std::string(spec.logical_name) + " -> 0";
        const std::string neg_label = std::string(spec.logical_name) + " -> -90";
        const std::string pos_label = std::string(spec.logical_name) + " -> +90";

        if (stagePose(home_label, makeJointOffsets(index, 0)) != 0) {
            return 1;
        }
        if (stagePose(neg_label, makeJointOffsets(index, kSmokeNegativeStep)) != 0) {
            return 1;
        }
        if (stagePose(pos_label, makeJointOffsets(index, kSmokePositiveStep)) != 0) {
            return 1;
        }
        if (stagePose(home_label, makeJointOffsets(index, 0)) != 0) {
            return 1;
        }

        return 0;
    };

    for (std::size_t i = 0; i < plan.count; ++i) {
        if (runJointSweep(plan.indices[i]) != 0) {
            motion_controller_.shutdown();
            return 1;
        }
    }

    motion_controller_.shutdown();
    std::cout << "[SMOKE] done" << std::endl;
    return 0;
}

int AppController::runLiveMarkerTest(const LiveTestOptions& options) {
    std::cout
        << "[LIVE_TEST] Starting live marker safety test mode\n"
        << "[LIVE_TEST] Camera index: " << options.camera_index << "\n"
        << "[LIVE_TEST] Expected marker ID: " << options.expected_marker_id << "\n"
        << "[LIVE_TEST] Auto operator ack: "
        << (options.auto_ack ? "ON" : "OFF") << "\n"
        << "[LIVE_TEST] Controls: a=arm, d=disarm, r=ack, q=quit\n"
        << "[LIVE_TEST] Starting in DISARMED setup mode\n"
        << "[LIVE_TEST] Guardian thresholds: freeze after "
        << kLiveFreezeBadFrameThreshold
        << " consecutive bad frames, recover after "
        << kLiveRecoverGoodFrameThreshold
        << " consecutive good frames.\n"
        << "[LIVE_TEST] Focus debug enabled: FOCUS_SCORE (Laplacian variance), "
           "higher usually means sharper marker edges.\n";

    MotionChannelMap channel_map{};
    channel_map.base = kBaseServoChannel;
    channel_map.lower = kLowerServoChannel;
    channel_map.upper = kUpperServoChannel;
    channel_map.gripper = kGripServoChannel;

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       channel_map)) {
        std::cerr << "[LIVE_TEST] Motion controller initialization failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    MotionControllerHardwareAdapter hardware(motion_controller_);
    VisionConfig vision_config;
    vision_config.expectedMarkerId = options.expected_marker_id;
    VisionProcessor vision_processor(vision_config);

    std::unique_ptr<GuardianStateMachine> guardian;
    std::unique_ptr<RobotInterlock> interlock;
    bool guardian_armed = false;
    bool motion_faulted = false;
    FreezeReason pending_reason = FreezeReason::UNKNOWN_FAULT;
    SafetyState current_vision_state = SafetyState::SAFE;
    bool frame_is_safe = false;

    auto resetEnforcementState = [&]() {
        pending_reason = FreezeReason::UNKNOWN_FAULT;
        guardian = std::make_unique<GuardianStateMachine>(
            kLiveFreezeBadFrameThreshold,
            kLiveRecoverGoodFrameThreshold);
        interlock = std::make_unique<RobotInterlock>(hardware);

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

    PhysicalButtonModule button_module;
    std::cout << "[LIVE_TEST] Physical button module: "
              << (button_module.available() ? "configured" : "disabled");
    const char* button_module_status = button_module.lastErrorString();
    if (button_module_status != nullptr &&
        std::string(button_module_status) != "no error") {
        std::cout << " (" << button_module_status << ")";
    }
    std::cout << std::endl;
    if (!button_module.available()) {
        std::cout << "[LIVE_TEST] Configure ARGUS_BUTTON_ARM_GPIO, "
                     "ARGUS_BUTTON_DISARM_GPIO, and/or ARGUS_BUTTON_ACK_GPIO to "
                     "enable physical operator buttons."
                  << std::endl;
    }

    auto freezeMotionBeforeModeChange = [&](const char* action_label) -> bool {
        interlock->onControlEvent(ControlEvent::FREEZE_NOW, FreezeReason::UNKNOWN_FAULT);
        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Unable to freeze motion before " << action_label
                      << ": " << motion_controller_.lastErrorString() << std::endl;
            return false;
        }

        return true;
    };

    auto requestArm = [&]() -> bool {
        if (guardian_armed) {
            std::cout << "[LIVE_TEST] ARM request ignored: guardian already armed."
                      << std::endl;
            return true;
        }

        if (!frame_is_safe) {
            std::cout << "[LIVE_TEST] ARM request rejected: current observed condition is "
                         "unsafe (vision="
                      << safetyStateToString(current_vision_state) << ")." << std::endl;
            return true;
        }

        if (!freezeMotionBeforeModeChange("arming")) {
            return false;
        }

        resetEnforcementState();
        guardian_armed = true;
        std::cout << "[LIVE_TEST] ARM accepted: guardian enforcement is now ACTIVE."
                  << std::endl;
        return true;
    };

    auto requestDisarm = [&]() -> bool {
        if (!guardian_armed) {
            std::cout << "[LIVE_TEST] DISARM request ignored: guardian already disarmed."
                      << std::endl;
            return true;
        }

        if (!freezeMotionBeforeModeChange("disarming")) {
            return false;
        }

        guardian_armed = false;
        resetEnforcementState();
        std::cout << "[LIVE_TEST] DISARM accepted: guardian enforcement is now INACTIVE "
                     "(setup/observation mode)."
                  << std::endl;
        return true;
    };

    auto requestAcknowledge = [&]() -> bool {
        if (!guardian_armed) {
            std::cout << "[LIVE_TEST] ACK request ignored: guardian is disarmed."
                      << std::endl;
            return true;
        }

        if (guardian->getState() != GuardianState::FROZEN_UNSAFE) {
            std::cout << "[LIVE_TEST] ACK request ignored: guardian is not frozen."
                      << std::endl;
            return true;
        }

        guardian->operatorAcknowledge();
        interlock->operatorAcknowledge();
        std::cout << "[LIVE_TEST] Operator acknowledge requested." << std::endl;
        return true;
    };

    auto requestFromButton = [&](PhysicalButtonEvent event) -> bool {
        std::cout << "[BUTTON] " << PhysicalButtonModule::eventToString(event)
                  << std::endl;
        switch (event) {
            case PhysicalButtonEvent::ARM_REQUEST:
                return requestArm();
            case PhysicalButtonEvent::DISARM_REQUEST:
                return requestDisarm();
            case PhysicalButtonEvent::ACK_REQUEST:
                return requestAcknowledge();
        }
        return true;
    };

    CameraCapture camera_capture(options.camera_index);
    cv::namedWindow(kLiveWindowName, cv::WINDOW_AUTOSIZE);

    FrameEvent frame_event;
    std::uint64_t frame_index = 0;
    bool processed_any_frame = false;
    int consecutive_capture_failures = 0;

    while (true) {
        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Interlock entered FAULT state (motion controller: "
                      << motion_controller_.lastErrorString() << ")" << std::endl;
            break;
        }

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

        current_vision_state = result.state;
        frame_is_safe = (current_vision_state == SafetyState::SAFE);
        if (!frame_is_safe) {
            pending_reason = mapSafetyToFreezeReason(current_vision_state);
        }

        std::string guardian_state_text = "DISARMED_SETUP";
        std::string interlock_state_text = "DISARMED";
        std::string freeze_reason_text = "N/A";

        if (guardian_armed) {
            guardian->processFrame(frame_is_safe ? FrameStatus::FRAME_GOOD
                                                 : FrameStatus::FRAME_BAD);
        }

        PhysicalButtonEvent button_event;
        while (button_module.poll(button_event)) {
            if (!requestFromButton(button_event)) {
                break;
            }
        }

        if (motion_faulted) {
            break;
        }

        if (guardian_armed && options.auto_ack &&
            guardian->getState() == GuardianState::FROZEN_UNSAFE) {
            guardian->operatorAcknowledge();
            interlock->operatorAcknowledge();
            std::cout << "[LIVE_TEST] Operator acknowledge sent automatically"
                      << std::endl;
        }

        if (guardian_armed) {
            interlock->guardianHeartbeat(static_cast<std::uint32_t>(frame_index));
            guardian_state_text = guardian->getCurrentStateString();
            interlock_state_text = interlockStateToString(interlock->state());
            freeze_reason_text = freezeReasonToString(interlock->freezeReason());
        }

        std::cout << "[LIVE_TEST] frame=" << frame_index
                  << " armed=" << (guardian_armed ? "YES" : "NO")
                  << " can_arm=" << (frame_is_safe ? "YES" : "NO")
                  << " vision=" << safetyStateToString(current_vision_state)
                  << " focus_score=" << formatFocusScore(focus_score)
                  << " focus=" << focus_quality
                  << " guardian=" << guardian_state_text
                  << " interlock=" << interlock_state_text
                  << " motion_ctrl="
                  << motionControllerStateToString(motion_controller_.outputState())
                  << " freeze_reason=" << freeze_reason_text
                  << " processing_us=" << result.processing_time.count()
                  << std::endl;

        if (interlock->state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[LIVE_TEST] Motion control fault detected: "
                      << motion_controller_.lastErrorString() << std::endl;
            break;
        }

        cv::Mat display_frame = frame_event.image_data.clone();
        const bool decision_is_safe = guardian_armed
                                          ? ((current_vision_state == SafetyState::SAFE) &&
                                             (guardian->getState() ==
                                              GuardianState::SAFE_MONITORING) &&
                                             interlock->motionAllowed())
                                          : (current_vision_state == SafetyState::SAFE);
        const cv::Scalar decision_color =
            decision_is_safe ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

        cv::putText(display_frame,
                    std::string("VISION: ") +
                        safetyStateToString(current_vision_state),
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
                        (guardian_armed ? "ARMED (d=disarm, r=ack)"
                                        : "DISARMED (a=arm, r=ack)") +
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

        cv::imshow(kLiveWindowName, display_frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            std::cout << "[LIVE_TEST] Exit requested from display window (q)." << std::endl;
            break;
        }

        if (key == 'r' || key == 'R') {
            if (!requestAcknowledge()) {
                break;
            }
        }

        if (key == 'a' || key == 'A') {
            if (!requestArm()) {
                break;
            }
        }

        if (key == 'd' || key == 'D') {
            if (!requestDisarm()) {
                break;
            }
        }

        ++frame_index;
    }

    cv::destroyWindow(kLiveWindowName);
    motion_controller_.shutdown();

    if (!processed_any_frame) {
        std::cerr << "[LIVE_TEST] No frames processed. Check camera availability."
                  << std::endl;
        return 1;
    }

    if (motion_faulted) {
        return 1;
    }

    std::cerr << "[LIVE_TEST] Frame stream ended." << std::endl;
    return 0;
}
