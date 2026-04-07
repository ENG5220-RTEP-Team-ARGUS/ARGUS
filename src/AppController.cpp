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
constexpr int kLiveFreezeBadFrameThreshold = 30;
constexpr int kLiveRecoverGoodFrameThreshold = 3;
constexpr int kSmokeFreezeBadFrameThreshold = 1;
constexpr int kSmokeRecoverGoodFrameThreshold = 1;
constexpr std::chrono::milliseconds kSmokeStepDwell{800};
constexpr std::chrono::milliseconds kSmokeTransitionPause{200};
constexpr std::uint16_t kSmokeNeutralPulseTicks = 300;
constexpr int kSmokeBaseMinOffset = -20;
constexpr int kSmokeBaseMaxOffset = 20;
constexpr int kSmokeLowerMinOffset = -15;
constexpr int kSmokeLowerMaxOffset = 15;
constexpr int kSmokeUpperMinOffset = -15;
constexpr int kSmokeUpperMaxOffset = 15;
constexpr int kSmokeGripMinOffset = -15;
constexpr int kSmokeGripMaxOffset = 15;
constexpr int kSmokeBasePositiveStep = 15;
constexpr int kSmokeBaseNegativeStep = -15;
constexpr int kSmokeOtherPositiveStep = 10;
constexpr int kSmokeOtherNegativeStep = -10;

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

struct SmokePose {
    const char* name;
    SmokeJointOffsets offsets;
    bool stage_while_frozen;
};

struct SmokeJointSpec {
    const char* logical_name;
    const char* mearm_name;
    std::uint8_t channel;
    int min_offset;
    int max_offset;
};

constexpr std::array<SmokeJointSpec, MotionController::kServoCount> kSmokeJointSpecs = {{
    {"base", "MeArm BASE", 0, kSmokeBaseMinOffset, kSmokeBaseMaxOffset},
    {"lower", "MeArm RIGHT", 1, kSmokeLowerMinOffset, kSmokeLowerMaxOffset},
    {"upper", "MeArm LEFT", 2, kSmokeUpperMinOffset, kSmokeUpperMaxOffset},
    {"grip", "MeArm CLAW", 3, kSmokeGripMinOffset, kSmokeGripMaxOffset},
}};

constexpr SmokeJointOffsets kSmokeHomePose{0, 0, 0, 0};
constexpr SmokeJointOffsets kSmokeBasePositivePose{kSmokeBasePositiveStep, 0, 0, 0};
constexpr SmokeJointOffsets kSmokeBaseNegativePose{kSmokeBaseNegativeStep, 0, 0, 0};
constexpr SmokeJointOffsets kSmokeLowerPositivePose{0, kSmokeOtherPositiveStep, 0, 0};
constexpr SmokeJointOffsets kSmokeLowerNegativePose{0, kSmokeOtherNegativeStep, 0, 0};
constexpr SmokeJointOffsets kSmokeUpperPositivePose{0, 0, kSmokeOtherPositiveStep, 0};
constexpr SmokeJointOffsets kSmokeUpperNegativePose{0, 0, kSmokeOtherNegativeStep, 0};
constexpr SmokeJointOffsets kSmokeGripPositivePose{0, 0, 0, kSmokeOtherPositiveStep};
constexpr SmokeJointOffsets kSmokeGripNegativePose{0, 0, 0, kSmokeOtherNegativeStep};

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

int AppController::runMotionSmokeTest() {
    std::cout
        << "[SMOKE_TEST] Starting motion smoke test mode\n"
        << "[SMOKE_TEST] Hardware path: AppController -> GuardianStateMachine -> "
           "RobotInterlock -> MotionController\n"
        << "[SMOKE_TEST] Logical joint home: base=0 lower=0 upper=0 grip=0\n"
        << "[SMOKE_TEST] PCA9685 neutral pulse for first run: "
        << kSmokeNeutralPulseTicks << " ticks\n"
        << "[SMOKE_TEST] Guardian thresholds: freeze after "
        << kSmokeFreezeBadFrameThreshold
        << " bad frame, recover after "
        << kSmokeRecoverGoodFrameThreshold
        << " good frame\n"
        << "[SMOKE_TEST] Conservative joint limits and hardware mapping:\n";

    for (const SmokeJointSpec& spec : kSmokeJointSpecs) {
        std::cout << "[SMOKE_TEST]   " << spec.logical_name
                  << " -> channel " << static_cast<int>(spec.channel)
                  << " -> " << spec.mearm_name
                  << " limits [" << spec.min_offset << ", " << spec.max_offset
                  << "]"
                  << std::endl;
    }

    std::cout
        << "[SMOKE_TEST] Sequence: HOME -> BASE +15 (freeze/recover) -> HOME -> "
           "BASE -15 -> HOME -> LOWER +10 -> HOME -> LOWER -10 -> HOME -> "
           "UPPER +10 -> HOME -> UPPER -10 -> HOME -> GRIP +10 -> HOME -> "
           "GRIP -10 -> HOME\n"
        << "[SMOKE_TEST] Every command is clamped to the offset windows above.\n";

    MotionChannelMap channel_map{};
    channel_map.base = 0;
    channel_map.lower = 1;
    channel_map.upper = 2;
    channel_map.gripper = 3;

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       channel_map)) {
        std::cerr << "[SMOKE_TEST] Motion controller initialization failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    MotionControllerHardwareAdapter hardware(motion_controller_);
    RobotInterlock interlock(hardware);
    GuardianStateMachine guardian(kSmokeFreezeBadFrameThreshold,
                                  kSmokeRecoverGoodFrameThreshold);

    guardian.setOnFreezeCallback([&]() {
        interlock.onControlEvent(ControlEvent::FREEZE_NOW, FreezeReason::UNKNOWN_FAULT);
    });

    guardian.setOnClearFreezeCallback([&]() {
        interlock.onControlEvent(ControlEvent::ALLOW_MOTION);
    });

    guardian.setOnStateChangeCallback([&](GuardianState from, GuardianState to) {
        std::cout << "[SMOKE_TEST] guardian " << guardian.stateToString(from) << " -> "
                  << guardian.stateToString(to) << std::endl;
    });

    auto fail = [&](const std::string& message) {
        std::cerr << "[SMOKE_TEST] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    auto logStatus = [&](const char* phase) {
        std::cout << "[SMOKE_TEST] phase=" << phase
                  << " guardian=" << guardian.getCurrentStateString()
                  << " interlock=" << interlockStateToString(interlock.state())
                  << " motion_ctrl="
                  << motionControllerStateToString(motion_controller_.outputState())
                  << " allowed=" << (interlock.motionAllowed() ? "YES" : "NO")
                  << std::endl;
    };

    auto stagePose = [&](const SmokePose& pose) {
        bool clamped = false;
        const SmokeJointOffsets bounded = clampSmokeOffsets(pose.offsets, clamped);
        const MeArmJointTargets targets = makeSmokeTargets(bounded, clamped);

        if (clamped) {
            std::cout << "[SMOKE_TEST] " << pose.name << " requested "
                      << formatOffsets(pose.offsets) << " clamped to "
                      << formatOffsets(bounded) << std::endl;
        }

        if (!motion_controller_.setTargets(targets)) {
            return fail(std::string("failed to stage pose ") + pose.name + ": " +
                        motion_controller_.lastErrorString());
        }

        std::cout << "[SMOKE_TEST] " << pose.name << " offsets "
                  << formatOffsets(bounded) << " targets "
                  << formatTargets(targets) << " ("
                  << (interlock.motionAllowed() ? "live" : "staged") << ")"
                  << std::endl;
        return 0;
    };

    auto freezeAndRecoverToPose = [&](const SmokePose& pose) {
        std::cout << "[SMOKE_TEST] cycle: freeze -> stage " << pose.name
                  << " -> recover" << std::endl;

        guardian.processFrame(FrameStatus::FRAME_BAD);
        if (guardian.getState() != GuardianState::FROZEN_UNSAFE ||
            interlock.state() != InterlockState::FROZEN) {
            return fail(std::string("guardian/interlock did not freeze before ") +
                        pose.name);
        }

        if (interlock.state() == InterlockState::FAULT ||
            motion_controller_.outputState() == MotionOutputState::FAULT) {
            return fail(std::string("fault while freezing before ") + pose.name);
        }

        logStatus("frozen");

        if (stagePose(pose) != 0) {
            return 1;
        }

        std::this_thread::sleep_for(kSmokeTransitionPause);

        guardian.operatorAcknowledge();
        interlock.operatorAcknowledge();
        guardian.processFrame(FrameStatus::FRAME_GOOD);
        if (interlock.state() == InterlockState::FAULT ||
            motion_controller_.outputState() == MotionOutputState::FAULT) {
            return fail(std::string("fault while recovering to ") + pose.name);
        }

        if (!interlock.motionAllowed() ||
            guardian.getState() != GuardianState::SAFE_MONITORING) {
            return fail(std::string("interlock did not re-enable cleanly for ") +
                        pose.name);
        }

        logStatus(pose.name);
        std::this_thread::sleep_for(kSmokeStepDwell);
        return 0;
    };

    auto liveMove = [&](const SmokePose& pose) {
        if (!interlock.motionAllowed() || interlock.state() != InterlockState::SAFE) {
            return fail(std::string("motion not allowed before live move ") + pose.name);
        }

        if (stagePose(pose) != 0) {
            return 1;
        }

        logStatus(pose.name);
        std::this_thread::sleep_for(kSmokeStepDwell);
        return 0;
    };

    logStatus("startup");

    const std::array<SmokePose, 17> smoke_sequence = {{
        {"HOME", kSmokeHomePose, false},
        {"BASE +15", kSmokeBasePositivePose, true},
        {"HOME", kSmokeHomePose, false},
        {"BASE -15", kSmokeBaseNegativePose, false},
        {"HOME", kSmokeHomePose, false},
        {"LOWER +10", kSmokeLowerPositivePose, false},
        {"HOME", kSmokeHomePose, false},
        {"LOWER -10", kSmokeLowerNegativePose, false},
        {"HOME", kSmokeHomePose, false},
        {"UPPER +10", kSmokeUpperPositivePose, false},
        {"HOME", kSmokeHomePose, false},
        {"UPPER -10", kSmokeUpperNegativePose, false},
        {"HOME", kSmokeHomePose, false},
        {"GRIP +10", kSmokeGripPositivePose, false},
        {"HOME", kSmokeHomePose, false},
        {"GRIP -10", kSmokeGripNegativePose, false},
        {"HOME", kSmokeHomePose, false},
    }};

    for (const SmokePose& pose : smoke_sequence) {
        const int result = pose.stage_while_frozen ? freezeAndRecoverToPose(pose)
                                                   : liveMove(pose);
        if (result != 0) {
            motion_controller_.shutdown();
            return result;
        }
    }

    motion_controller_.shutdown();
    std::cout << "[SMOKE_TEST] Motion smoke test complete." << std::endl;
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
    channel_map.base = 0;
    channel_map.lower = 1;
    channel_map.upper = 2;
    channel_map.gripper = 3;

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
