#include "AppController.hpp"

#include "CameraCapture.hpp"
#include "GuardianStateMachine.hpp"
#include "PhysicalButtonModule.hpp"
#include "RealtimeTimer.hpp"
#include "RobotInterlock.hpp"
#include "VisionProcessor.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

constexpr const char* kLiveWindowName = "ARGUS Live Test";
constexpr const char* kDemoWindowName = "ARGUS Full Demo";
constexpr const char* kDefaultI2cDevicePath = "/dev/i2c-1";
constexpr std::uint8_t kDefaultPca9685Address = 0x40;
constexpr float kDefaultPwmFrequencyHz = 50.0f;
constexpr std::uint8_t kBaseServoChannel = 0;
constexpr std::uint8_t kLowerServoChannel = 4;
constexpr std::uint8_t kUpperServoChannel = 8;
constexpr std::uint8_t kGripServoChannel = 12;
constexpr MotionChannelMap kMotionChannelMap{
    kBaseServoChannel,
    kLowerServoChannel,
    kUpperServoChannel,
    kGripServoChannel};
constexpr int kLiveFreezeBadFrameThreshold = 30;
constexpr int kLiveRecoverGoodFrameThreshold = 3;
constexpr std::chrono::milliseconds kSmokeStepDwell{3000};
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
volatile std::sig_atomic_t g_interactive_servo_stop_requested = 0;
constexpr std::chrono::milliseconds kMotionHomeSettleDwell{2000};
constexpr int kDemoFreezeBadFrameThreshold = 1;
constexpr int kDemoRecoverGoodFrameThreshold = 3;
constexpr std::chrono::milliseconds kDemoStepDwell{1000};
constexpr int kDemoBaseMinOffset = -60;
constexpr int kDemoBaseMaxOffset = 60;
constexpr int kDemoLowerMinOffset = -60;
constexpr int kDemoLowerMaxOffset = 60;
constexpr int kDemoUpperMinOffset = -60;
constexpr int kDemoUpperMaxOffset = 60;
constexpr int kDemoGripMinOffset = -60;
constexpr int kDemoGripMaxOffset = 60;
constexpr int kDemoBaseStep = 60;
constexpr int kDemoLowerStep = 60;
constexpr int kDemoUpperStep = 60;
constexpr int kDemoGripStep = 60;
constexpr std::chrono::milliseconds kCaptureRetryBackoff{50};

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

void handleInteractiveServoSignal(int) {
    g_interactive_servo_stop_requested = 1;
}

bool waitForRealtimeDelay(std::chrono::nanoseconds delay,
                          std::string& error_message) {
    if (delay.count() <= 0) {
        error_message.clear();
        return true;
    }

    std::mutex mutex;
    std::condition_variable condition;
    bool fired = false;

    RealtimeTimer timer;
    if (!timer.startOneShot(delay, [&]() {
            std::lock_guard<std::mutex> lock(mutex);
            fired = true;
            condition.notify_one();
        })) {
        error_message = timer.lastErrorString();
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&]() { return fired; });
    error_message.clear();
    return true;
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

struct JointPulseCalibration {
    std::uint16_t neg90_ticks;
    std::uint16_t zero_ticks;
    std::uint16_t pos90_ticks;
};

struct JointCalibrationMarks {
    bool has_neg90{false};
    bool has_zero{false};
    bool has_pos90{false};
    std::uint16_t neg90_ticks{0};
    std::uint16_t zero_ticks{0};
    std::uint16_t pos90_ticks{0};
};

constexpr std::array<SmokeJointSpec, MotionController::kServoCount> kSmokeJointSpecs = {{
    {"base", "MeArm BASE", "yaw left/right", kMotionChannelMap.base, kSmokeBaseMinOffset, kSmokeBaseMaxOffset},
    {"lower", "MeArm LEFT", "raise/lower", kMotionChannelMap.lower, kSmokeLowerMinOffset, kSmokeLowerMaxOffset},
    {"upper", "MeArm RIGHT", "bend/extend", kMotionChannelMap.upper, kSmokeUpperMinOffset, kSmokeUpperMaxOffset},
    {"grip", "MeArm CLAW", "open/close", kMotionChannelMap.gripper, kSmokeGripMinOffset, kSmokeGripMaxOffset},
}};

constexpr std::array<JointPulseCalibration, MotionController::kServoCount>
    kJointPulseCalibration = {{
        {100, 300, 500},
        {100, 300, 500},
        {100, 290, 500},
        {100, 300, 500},
    }};

constexpr SmokeJointOffsets kSmokeHomePose{0, 0, 0, 0};

struct DemoPoseStep {
    const char* name;
    SmokeJointOffsets offsets;
};

constexpr DemoPoseStep kDemoHomeStep{"HOME", kSmokeHomePose};

constexpr std::array<DemoPoseStep, 12> kDemoSequence = {{
    {"BASE +60", {kDemoBaseStep, 0, 0, 0}},
    {"BASE -60", {-kDemoBaseStep, 0, 0, 0}},
    {"HOME", {0, 0, 0, 0}},
    {"LOWER +60", {0, kDemoLowerStep, 0, 0}},
    {"LOWER -60", {0, -kDemoLowerStep, 0, 0}},
    {"HOME", {0, 0, 0, 0}},
    {"UPPER +60", {0, 0, kDemoUpperStep, 0}},
    {"UPPER -60", {0, 0, -kDemoUpperStep, 0}},
    {"HOME", {0, 0, 0, 0}},
    {"GRIP +60", {0, 0, 0, kDemoGripStep}},
    {"GRIP -60", {0, 0, 0, -kDemoGripStep}},
    {"HOME", {0, 0, 0, 0}},
}};

std::string toLowerCopy(std::string text);

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

const char* smokeJointIndexToString(std::size_t index) {
    switch (index) {
        case 0:
            return "base";
        case 1:
            return "lower";
        case 2:
            return "upper";
        case 3:
            return "grip";
        default:
            return "unknown";
    }
}

bool smokeJointIndexFromName(const std::string& name, std::size_t& index) {
    const std::string lower = toLowerCopy(name);
    if (lower == "base") {
        index = 0;
        return true;
    }
    if (lower == "lower") {
        index = 1;
        return true;
    }
    if (lower == "upper") {
        index = 2;
        return true;
    }
    if (lower == "grip" || lower == "gripper") {
        index = 3;
        return true;
    }
    return false;
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

SmokeJointOffsets clampOffsetsToWindow(const SmokeJointOffsets& requested,
                                       int base_min,
                                       int base_max,
                                       int lower_min,
                                       int lower_max,
                                       int upper_min,
                                       int upper_max,
                                       int grip_min,
                                       int grip_max,
                                       bool& clamped) {
    SmokeJointOffsets bounded = requested;
    bounded.base = clampOffsetValue(requested.base, base_min, base_max, clamped);
    bounded.lower = clampOffsetValue(requested.lower, lower_min, lower_max, clamped);
    bounded.upper = clampOffsetValue(requested.upper, upper_min, upper_max, clamped);
    bounded.grip = clampOffsetValue(requested.grip, grip_min, grip_max, clamped);
    return bounded;
}

SmokeJointOffsets clampSmokeOffsets(const SmokeJointOffsets& requested,
                                    bool& clamped) {
    return clampOffsetsToWindow(requested,
                                kSmokeBaseMinOffset,
                                kSmokeBaseMaxOffset,
                                kSmokeLowerMinOffset,
                                kSmokeLowerMaxOffset,
                                kSmokeUpperMinOffset,
                                kSmokeUpperMaxOffset,
                                kSmokeGripMinOffset,
                                kSmokeGripMaxOffset,
                                clamped);
}

SmokeJointOffsets clampDemoOffsets(const SmokeJointOffsets& requested,
                                   bool& clamped) {
    return clampOffsetsToWindow(requested,
                                kDemoBaseMinOffset,
                                kDemoBaseMaxOffset,
                                kDemoLowerMinOffset,
                                kDemoLowerMaxOffset,
                                kDemoUpperMinOffset,
                                kDemoUpperMaxOffset,
                                kDemoGripMinOffset,
                                kDemoGripMaxOffset,
                                clamped);
}

std::uint16_t logicalAngleToPulseTicks(std::size_t joint_index,
                                       int angle_degrees,
                                       bool& clamped) {
    const JointPulseCalibration& calibration =
        kJointPulseCalibration.at(joint_index);
    const int zero_ticks = static_cast<int>(calibration.zero_ticks);
    const int delta_ticks = angle_degrees >= 0
                                ? static_cast<int>(calibration.pos90_ticks) -
                                      zero_ticks
                                : zero_ticks -
                                      static_cast<int>(calibration.neg90_ticks);
    const int raw = zero_ticks +
                    static_cast<int>(std::lround(
                        static_cast<double>(delta_ticks) *
                        (static_cast<double>(angle_degrees) / 90.0)));
    const int bounded = std::clamp(
        raw, 0, static_cast<int>(MotionController::kMaxPulseTicks));
    if (bounded != raw) {
        clamped = true;
    }
    return static_cast<std::uint16_t>(bounded);
}

std::uint16_t clampPulseTicks(int raw_ticks, bool& clamped) {
    const int bounded = std::clamp(raw_ticks,
                                   0,
                                   static_cast<int>(MotionController::kMaxPulseTicks));
    if (bounded != raw_ticks) {
        clamped = true;
    }
    return static_cast<std::uint16_t>(bounded);
}

MeArmJointTargets makeSmokeTargets(const SmokeJointOffsets& requested,
                                   bool& clamped) {
    const SmokeJointOffsets bounded = clampSmokeOffsets(requested, clamped);
    return {
        logicalAngleToPulseTicks(0, bounded.base, clamped),
        logicalAngleToPulseTicks(1, bounded.lower, clamped),
        logicalAngleToPulseTicks(2, bounded.upper, clamped),
        logicalAngleToPulseTicks(3, bounded.grip, clamped)};
}

MeArmJointTargets makeDemoTargets(const SmokeJointOffsets& requested,
                                  bool& clamped) {
    const SmokeJointOffsets bounded = clampDemoOffsets(requested, clamped);
    return {
        logicalAngleToPulseTicks(0, bounded.base, clamped),
        logicalAngleToPulseTicks(1, bounded.lower, clamped),
        logicalAngleToPulseTicks(2, bounded.upper, clamped),
        logicalAngleToPulseTicks(3, bounded.grip, clamped)};
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

std::uint16_t pulseForJoint(const MeArmJointTargets& targets, std::size_t index) {
    switch (index) {
        case 0:
            return targets.base_ticks;
        case 1:
            return targets.lower_ticks;
        case 2:
            return targets.upper_ticks;
        case 3:
            return targets.gripper_ticks;
        default:
            return 0;
    }
}

void setPulseForJoint(MeArmJointTargets& targets,
                      std::size_t index,
                      std::uint16_t ticks) {
    switch (index) {
        case 0:
            targets.base_ticks = ticks;
            break;
        case 1:
            targets.lower_ticks = ticks;
            break;
        case 2:
            targets.upper_ticks = ticks;
            break;
        case 3:
            targets.gripper_ticks = ticks;
            break;
        default:
            break;
    }
}

bool parseCalibrationPoint(const std::string& text,
                           const char*& label,
                           int& nominal_degrees) {
    const std::string lower = toLowerCopy(text);
    if (lower == "-90" || lower == "neg90" || lower == "minus90") {
        label = "-90";
        nominal_degrees = -90;
        return true;
    }
    if (lower == "0" || lower == "zero" || lower == "center" || lower == "centre") {
        label = "0";
        nominal_degrees = 0;
        return true;
    }
    if (lower == "+90" || lower == "90" || lower == "pos90" || lower == "plus90") {
        label = "+90";
        nominal_degrees = 90;
        return true;
    }
    return false;
}

std::string formatCalibrationMarks(std::size_t joint_index,
                                   const JointCalibrationMarks& marks) {
    std::ostringstream oss;
    oss << smokeJointIndexToString(joint_index) << ": "
        << "-90=";
    if (marks.has_neg90) {
        oss << marks.neg90_ticks;
    } else {
        oss << "unset";
    }
    oss << "  0=";
    if (marks.has_zero) {
        oss << marks.zero_ticks;
    } else {
        oss << "unset";
    }
    oss << "  +90=";
    if (marks.has_pos90) {
        oss << marks.pos90_ticks;
    } else {
        oss << "unset";
    }
    return oss.str();
}

std::string buildCalibrationSummary(
    const std::array<JointCalibrationMarks, MotionController::kServoCount>& marks) {
    std::ostringstream oss;
    oss << "# ARGUS servo calibration summary\n"
        << "# This report is informational only; values are not applied automatically.\n"
        << "# Format: joint  -90=<ticks>  0=<ticks>  +90=<ticks>\n";
    for (std::size_t i = 0; i < marks.size(); ++i) {
        oss << formatCalibrationMarks(i, marks[i]) << "\n";
    }
    return oss.str();
}

std::string toLowerCopy(std::string text) {
    std::transform(text.begin(),
                   text.end(),
                   text.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    return text;
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

int AppController::runButtonTest() {
    PhysicalButtonModule button_module;
    std::cout << "[BUTTON_TEST] physical button test\n"
              << "[BUTTON_TEST] press Ctrl+C to stop\n"
              << "[BUTTON_TEST] module: "
              << (button_module.available() ? "configured" : "disabled");
    if (button_module.available()) {
        std::cout << " (" << button_module.statusString() << ")";
    } else {
        std::cout << " (" << button_module.lastErrorString() << ")";
    }
    std::cout << std::endl;

    if (!button_module.available()) {
        return 1;
    }

    bool pressed = false;
    if (!button_module.readAcknowledgePressed(pressed)) {
        std::cerr << "[BUTTON_TEST] initial read failed: "
                  << button_module.lastErrorString() << std::endl;
        return 1;
    }

    bool last_pressed = pressed;
    std::cout << "[BUTTON_TEST] state="
              << (pressed ? "PRESSED" : "RELEASED") << std::endl;

    while (true) {
        if (!button_module.readAcknowledgePressed(pressed)) {
            std::cerr << "[BUTTON_TEST] read failed: "
                      << button_module.lastErrorString() << std::endl;
            return 1;
        }

        if (pressed != last_pressed) {
            last_pressed = pressed;
            std::cout << "[BUTTON_TEST] state="
                      << (pressed ? "PRESSED" : "RELEASED") << std::endl;
        }

        PhysicalButtonEvent event;
        while (button_module.poll(event)) {
            std::cout << "[BUTTON_TEST] event="
                      << PhysicalButtonModule::eventToString(event) << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

int AppController::runServoCalibration() {
    std::cout
        << "[CAL] servo calibration console\n"
        << "[CAL] raw PCA9685 pulse control in ticks\n"
        << "[CAL] commands: base <ticks>, lower <ticks>, upper <ticks>, grip <ticks>\n"
        << "[CAL] nudges: base +5, base -5, grip +10, ...\n"
        << "[CAL] mark: mark <joint> <-90|0|+90>\n"
        << "[CAL] extras: home, status, summary, write [path], help\n"
        << "[CAL] quit: Ctrl+C or type 'exit'\n";

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       kMotionChannelMap)) {
        std::cerr << "[CAL] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[CAL] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    bool home_clamped = false;
    MeArmJointTargets current_targets =
        makeSmokeTargets(kSmokeHomePose, home_clamped);

    if (!motion_controller_.setTargets(current_targets)) {
        return fail(std::string("failed to stage home pulse set: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::array<JointCalibrationMarks, MotionController::kServoCount> marks{};
    std::cout << "[CAL] home " << formatTargets(current_targets) << std::endl;

    const auto previous_sigint = std::signal(SIGINT, handleInteractiveServoSignal);
    const auto previous_sigterm = std::signal(SIGTERM, handleInteractiveServoSignal);
    g_interactive_servo_stop_requested = 0;

    auto restoreSignals = [&]() {
        std::signal(SIGINT, previous_sigint);
        std::signal(SIGTERM, previous_sigterm);
    };

    auto applyTargets = [&](const MeArmJointTargets& requested,
                            const std::string& label) -> bool {
        MeArmJointTargets bounded = requested;
        bool clamped = false;
        bounded.base_ticks = clampPulseTicks(static_cast<int>(requested.base_ticks), clamped);
        bounded.lower_ticks = clampPulseTicks(static_cast<int>(requested.lower_ticks), clamped);
        bounded.upper_ticks = clampPulseTicks(static_cast<int>(requested.upper_ticks), clamped);
        bounded.gripper_ticks = clampPulseTicks(static_cast<int>(requested.gripper_ticks), clamped);

        if (!motion_controller_.setTargets(bounded)) {
            (void)fail(std::string("failed to set ") + label + ": " +
                       motion_controller_.lastErrorString());
            return false;
        }

        current_targets = bounded;
        std::cout << "[CAL] " << label << " " << formatTargets(current_targets);
        if (clamped) {
            std::cout << " [clamped]";
        }
        std::cout << std::endl;
        return true;
    };

    auto writeSummaryToFile = [&](const std::string& path) -> bool {
        std::ofstream out(path);
        if (!out.is_open()) {
            std::cerr << "[CAL] write failed: " << path << std::endl;
            return false;
        }
        out << buildCalibrationSummary(marks);
        out.close();
        std::cout << "[CAL] wrote " << path << std::endl;
        return true;
    };

    while (!g_interactive_servo_stop_requested) {
        std::cout << "cal> " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) {
            if (g_interactive_servo_stop_requested || std::cin.eof()) {
                break;
            }
            restoreSignals();
            return fail("stdin read failed");
        }

        std::istringstream iss(line);
        std::string command;
        if (!(iss >> command)) {
            continue;
        }

        command = toLowerCopy(command);
        if (command == "exit" || command == "quit") {
            break;
        }

        if (command == "help") {
            std::cout
                << "[CAL] commands: <joint> <ticks>, <joint> +/-<delta>, mark <joint> <-90|0|+90>, home, status, summary, write [path], exit"
                << std::endl;
            continue;
        }

        if (command == "home") {
            bool clamped = false;
            const MeArmJointTargets home_targets =
                makeSmokeTargets(kSmokeHomePose, clamped);
            if (!applyTargets(home_targets, "home")) {
                restoreSignals();
                return 1;
            }
            continue;
        }

        if (command == "status") {
            std::cout << "[CAL] " << formatTargets(current_targets) << std::endl;
            for (std::size_t i = 0; i < marks.size(); ++i) {
                std::cout << "[CAL] " << formatCalibrationMarks(i, marks[i])
                          << std::endl;
            }
            continue;
        }

        if (command == "summary") {
            std::cout << buildCalibrationSummary(marks);
            continue;
        }

        if (command == "write") {
            std::string path;
            if (!(iss >> path)) {
                path = "config/servo_calibration_latest.txt";
            }
            (void)writeSummaryToFile(path);
            continue;
        }

        if (command == "mark") {
            std::string joint_name;
            std::string point_name;
            if (!(iss >> joint_name >> point_name)) {
                std::cout << "[CAL] expected: mark <joint> <-90|0|+90>" << std::endl;
                continue;
            }

            std::size_t joint_index = 0;
            if (!smokeJointIndexFromName(joint_name, joint_index)) {
                std::cout << "[CAL] unknown joint: " << joint_name << std::endl;
                continue;
            }

            const char* point_label = nullptr;
            int nominal_degrees = 0;
            if (!parseCalibrationPoint(point_name, point_label, nominal_degrees)) {
                std::cout << "[CAL] unknown calibration point: " << point_name
                          << std::endl;
                continue;
            }

            JointCalibrationMarks& joint_marks = marks[joint_index];
            const std::uint16_t current_ticks = pulseForJoint(current_targets, joint_index);
            if (nominal_degrees < 0) {
                joint_marks.has_neg90 = true;
                joint_marks.neg90_ticks = current_ticks;
            } else if (nominal_degrees > 0) {
                joint_marks.has_pos90 = true;
                joint_marks.pos90_ticks = current_ticks;
            } else {
                joint_marks.has_zero = true;
                joint_marks.zero_ticks = current_ticks;
            }

            std::cout << "[CAL] marked " << smokeJointIndexToString(joint_index)
                      << " " << point_label << "=" << current_ticks << std::endl;
            continue;
        }

        std::size_t joint_index = 0;
        if (!smokeJointIndexFromName(command, joint_index)) {
            std::cout << "[CAL] unknown command or joint: " << command << std::endl;
            continue;
        }

        std::string value_text;
        if (!(iss >> value_text)) {
            std::cout << "[CAL] expected: <joint> <ticks>" << std::endl;
            continue;
        }

        int raw_value = 0;
        try {
            std::size_t parsed = 0;
            raw_value = std::stoi(value_text, &parsed);
            if (parsed != value_text.size()) {
                throw std::invalid_argument("trailing characters");
            }
        } catch (const std::exception&) {
            std::cout << "[CAL] invalid tick value: " << value_text << std::endl;
            continue;
        }

        MeArmJointTargets requested = current_targets;
        const int requested_ticks =
            static_cast<int>(pulseForJoint(current_targets, joint_index)) + 0;
        bool value_clamped = false;
        if (!value_text.empty() && (value_text[0] == '+' || value_text[0] == '-')) {
            setPulseForJoint(requested,
                             joint_index,
                             clampPulseTicks(requested_ticks + raw_value,
                                             value_clamped));
        } else {
            setPulseForJoint(requested,
                             joint_index,
                             clampPulseTicks(raw_value, value_clamped));
        }

        std::ostringstream label;
        label << smokeJointIndexToString(joint_index) << "="
              << pulseForJoint(requested, joint_index);
        if (!applyTargets(requested, label.str())) {
            restoreSignals();
            return 1;
        }
    }

    restoreSignals();
    std::cout << buildCalibrationSummary(marks);
    motion_controller_.shutdown();
    std::cout << "[CAL] done" << std::endl;
    return 0;
}

int AppController::runInteractiveServoConsole() {
    std::cout
        << "[SERVO] interactive servo console\n"
        << "[SERVO] commands: base <deg>, lower <deg>, upper <deg>, grip <deg>\n"
        << "[SERVO] extras: home, status, help\n"
        << "[SERVO] range: -90..+90 logical degrees\n"
        << "[SERVO] quit: Ctrl+C or type 'exit'\n";

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       kMotionChannelMap)) {
        std::cerr << "[SERVO] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[SERVO] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    SmokeJointOffsets current_offsets = kSmokeHomePose;
    bool initial_clamped = false;
    const MeArmJointTargets home_targets =
        makeSmokeTargets(current_offsets, initial_clamped);

    if (!motion_controller_.setTargets(home_targets)) {
        return fail(std::string("failed to stage home pose: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::cout << "[SERVO] home " << formatOffsets(current_offsets) << std::endl;

    const auto previous_sigint = std::signal(SIGINT, handleInteractiveServoSignal);
    const auto previous_sigterm = std::signal(SIGTERM, handleInteractiveServoSignal);
    g_interactive_servo_stop_requested = 0;

    auto restoreSignals = [&]() {
        std::signal(SIGINT, previous_sigint);
        std::signal(SIGTERM, previous_sigterm);
    };

    auto applyOffsets = [&](const SmokeJointOffsets& requested,
                            const std::string& label) -> bool {
        bool clamped = false;
        const SmokeJointOffsets bounded = clampSmokeOffsets(requested, clamped);
        const MeArmJointTargets targets = makeSmokeTargets(bounded, clamped);

        if (!motion_controller_.setTargets(targets)) {
            (void)fail(std::string("failed to set ") + label + ": " +
                       motion_controller_.lastErrorString());
            return false;
        }

        current_offsets = bounded;
        std::cout << "[SERVO] " << label << " " << formatOffsets(current_offsets);
        if (clamped) {
            std::cout << " [clamped]";
        }
        std::cout << std::endl;
        return true;
    };

    while (!g_interactive_servo_stop_requested) {
        std::cout << "servo> " << std::flush;

        std::string line;
        if (!std::getline(std::cin, line)) {
            if (g_interactive_servo_stop_requested) {
                break;
            }
            if (std::cin.eof()) {
                break;
            }
            restoreSignals();
            return fail("stdin read failed");
        }

        std::istringstream iss(line);
        std::string command;
        if (!(iss >> command)) {
            continue;
        }

        command = toLowerCopy(command);
        if (command == "exit" || command == "quit") {
            break;
        }

        if (command == "help") {
            std::cout
                << "[SERVO] commands: base <deg>, lower <deg>, upper <deg>, grip <deg>, home, status, exit"
                << std::endl;
            continue;
        }

        if (command == "status") {
            std::cout << "[SERVO] " << formatOffsets(current_offsets)
                      << std::endl;
            continue;
        }

        if (command == "home") {
            if (!applyOffsets(kSmokeHomePose, "home")) {
                restoreSignals();
                return 1;
            }
            continue;
        }

        int angle = 0;
        if (!(iss >> angle)) {
            std::cout << "[SERVO] expected: <joint> <angle>" << std::endl;
            continue;
        }

        SmokeJointOffsets requested = current_offsets;
        if (command == "base") {
            requested.base = angle;
        } else if (command == "lower") {
            requested.lower = angle;
        } else if (command == "upper") {
            requested.upper = angle;
        } else if (command == "grip" || command == "gripper") {
            requested.grip = angle;
        } else {
            std::cout << "[SERVO] unknown joint: " << command << std::endl;
            continue;
        }

        std::ostringstream label;
        label << command << "=" << angle;
        if (!applyOffsets(requested, label.str())) {
            restoreSignals();
            return 1;
        }
    }

    restoreSignals();
    motion_controller_.shutdown();
    std::cout << "[SERVO] done" << std::endl;
    return 0;
}

int AppController::runMotionHomePose() {
    std::cout << "[HOME] setting all joints to 0" << std::endl;

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       kMotionChannelMap)) {
        std::cerr << "[HOME] init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    auto fail = [&](const std::string& message) {
        std::cerr << "[HOME] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    bool clamped = false;
    const MeArmJointTargets home_targets =
        makeSmokeTargets(kSmokeHomePose, clamped);

    if (!motion_controller_.setTargets(home_targets)) {
        return fail(std::string("failed to stage home pose: ") +
                    motion_controller_.lastErrorString());
    }

    if (!motion_controller_.enable()) {
        return fail(std::string("failed to enable motion: ") +
                    motion_controller_.lastErrorString());
    }

    std::cout << "[HOME] pose=HOME wait=2s";
    if (clamped) {
        std::cout << " [clamped]";
    }
    std::cout << std::endl;

    std::string timer_error;
    if (!waitForRealtimeDelay(kMotionHomeSettleDwell, timer_error)) {
        return fail(std::string("home dwell timer failed: ") + timer_error);
    }

    motion_controller_.shutdown();
    std::cout << "[HOME] done" << std::endl;
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

        std::string timer_error;
        if (!waitForRealtimeDelay(kSmokeStepDwell, timer_error)) {
            return fail(std::string("smoke dwell timer failed: ") + timer_error);
        }
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
    {
        std::string timer_error;
        if (!waitForRealtimeDelay(kSmokeStepDwell, timer_error)) {
            return fail(std::string("initial smoke dwell timer failed: ") +
                        timer_error);
        }
    }

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

int AppController::runFullPipelineDemo(const LiveTestOptions& options) {
    if (options.auto_ack) {
        std::cerr << "[DEMO] --auto-ack is not supported in full-demo mode."
                  << std::endl;
        return 1;
    }

    std::cout
        << "[DEMO] full pipeline hardware demo\n"
        << "[DEMO] camera=" << options.camera_index
        << " marker=" << options.expected_marker_id << "\n"
        << "[DEMO] map base=0 lower=4 upper=8 grip=12\n"
        << "[DEMO] sequence HOME -> BASE +/-60 -> LOWER +/-60 -> UPPER +/-60 -> GRIP +/-60\n"
        << "[DEMO] freeze after 1 bad frame, recover after 3 good frames\n"
        << "[DEMO] safe scene required before arm/start\n"
        << "[DEMO] physical button = continue (arm/start or resume)\n"
        << "[DEMO] controls: a=continue, r=continue, q=quit\n";

    if (!motion_controller_.initialise(kDefaultI2cDevicePath,
                                       kDefaultPca9685Address,
                                       kDefaultPwmFrequencyHz,
                                       kMotionChannelMap)) {
        std::cerr << "[DEMO] motion init failed: "
                  << motion_controller_.lastErrorString() << std::endl;
        return 1;
    }

    MotionControllerHardwareAdapter hardware(motion_controller_);
    VisionConfig vision_config;
    vision_config.expectedMarkerId = options.expected_marker_id;
    VisionProcessor vision_processor(vision_config);
    CameraCapture camera_capture(options.camera_index);
    PhysicalButtonModule button_module;
    std::cout << "[DEMO] physical button module: "
              << (button_module.available() ? "configured" : "disabled");
    if (button_module.available()) {
        std::cout << " (" << button_module.statusString() << ")";
    } else {
        std::cout << " (" << button_module.lastErrorString() << ")";
    }
    std::cout << std::endl;
    if (!button_module.available()) {
        std::cerr << "[DEMO] physical ACK button unavailable: "
                  << button_module.lastErrorString() << std::endl;
        motion_controller_.shutdown();
        return 1;
    }

    GuardianStateMachine guardian(kDemoFreezeBadFrameThreshold,
                                  kDemoRecoverGoodFrameThreshold);
    RobotInterlock interlock(hardware);

    bool motion_gate_open = false;
    bool demo_armed = false;
    bool waiting_for_ack = false;
    bool waiting_for_ack_announced = false;
    bool scene_is_safe = false;
    bool scene_was_safe = false;
    bool scene_state_known = false;
    bool motion_faulted = false;
    bool processed_any_frame = false;
    SafetyState current_vision_state = SafetyState::SAFE;
    FreezeReason pending_reason = FreezeReason::UNKNOWN_FAULT;
    bool home_pose_staged = false;
    std::string current_pose_name = "NONE";
    std::size_t next_step_index = 0;
    RealtimeTimer demo_step_timer;
    std::atomic<bool> demo_step_due{false};

    auto fail = [&](const std::string& message) {
        std::cerr << "[DEMO] " << message << std::endl;
        motion_controller_.shutdown();
        return 1;
    };

    auto stagePose = [&](const DemoPoseStep& step) -> bool {
        bool clamped = false;
        const MeArmJointTargets targets = makeDemoTargets(step.offsets, clamped);

        if (!motion_controller_.setTargets(targets)) {
            (void)fail(std::string("failed to stage pose ") + step.name + ": " +
                       motion_controller_.lastErrorString());
            return false;
        }

        current_pose_name = step.name;
        std::cout << "[DEMO] pose=" << step.name;
        if (clamped) {
            std::cout << " [clamped]";
        }
        std::cout << " wait=1s" << std::endl;
        return true;
    };

    auto stopDemoStepTimer = [&]() {
        demo_step_timer.stop();
        demo_step_due.store(false, std::memory_order_relaxed);
    };

    auto startDemoStepTimer = [&]() -> bool {
        demo_step_due.store(false, std::memory_order_relaxed);
        if (!demo_step_timer.startPeriodic(
                kDemoStepDwell,
                [&]() { demo_step_due.store(true, std::memory_order_relaxed); })) {
            (void)fail(std::string("demo step timer failed: ") +
                       demo_step_timer.lastErrorString());
            return false;
        }
        return true;
    };

    auto requestArm = [&]() -> bool {
        if (demo_armed) {
            std::cout << "[DEMO] ARM ignored: already armed" << std::endl;
            return true;
        }

        if (!scene_is_safe) {
            std::cout << "[DEMO] ARM rejected: scene not safe" << std::endl;
            return true;
        }

        if (!home_pose_staged) {
            if (!stagePose(kDemoHomeStep)) {
                return false;
            }
            home_pose_staged = true;
        }

        interlock.onControlEvent(ControlEvent::FREEZE_NOW, FreezeReason::UNKNOWN_FAULT);
        if (interlock.state() == InterlockState::FAULT) {
            (void)fail(std::string("unable to prepare motion path for arm: ") +
                       motion_controller_.lastErrorString());
            return false;
        }

        interlock.operatorAcknowledge();
        interlock.onControlEvent(ControlEvent::ALLOW_MOTION);
        if (interlock.state() == InterlockState::FAULT) {
            (void)fail(std::string("unable to enable motion on arm: ") +
                       motion_controller_.lastErrorString());
            return false;
        }

        if (!startDemoStepTimer()) {
            return false;
        }

        demo_armed = true;
        motion_gate_open = true;
        waiting_for_ack = false;
        waiting_for_ack_announced = false;
        next_step_index = 0;
        std::cout << "[DEMO] ARM accepted -> running" << std::endl;
        return true;
    };

    auto requestAcknowledge = [&]() -> bool {
        if (!demo_armed) {
            std::cout << "[DEMO] ACK ignored: demo not armed" << std::endl;
            return true;
        }

        if (!scene_is_safe) {
            std::cout << "[DEMO] ACK ignored: scene not safe" << std::endl;
            return true;
        }

        if (guardian.getState() != GuardianState::FROZEN_UNSAFE) {
            std::cout << "[DEMO] ACK ignored: not frozen" << std::endl;
            return true;
        }

        guardian.operatorAcknowledge();
        interlock.operatorAcknowledge();
        waiting_for_ack = false;
        waiting_for_ack_announced = false;
        std::cout << "[DEMO] ACK accepted -> recovery" << std::endl;
        return interlock.state() != InterlockState::FAULT;
    };

    auto requestContinue = [&]() -> bool {
        return demo_armed ? requestAcknowledge() : requestArm();
    };

    auto requestFromButton = [&](PhysicalButtonEvent event) -> bool {
        std::cout << "[BUTTON] " << PhysicalButtonModule::eventToString(event)
                  << std::endl;
        switch (event) {
            case PhysicalButtonEvent::ACK_REQUEST:
                return requestContinue();
            case PhysicalButtonEvent::ARM_REQUEST:
                return requestContinue();
            case PhysicalButtonEvent::DISARM_REQUEST:
                std::cout << "[DEMO] button ignored" << std::endl;
                return true;
        }
        return true;
    };

    guardian.setOnFreezeCallback([&]() {
        stopDemoStepTimer();
        motion_gate_open = false;
        waiting_for_ack = false;
        waiting_for_ack_announced = false;

        std::cout << "[DEMO] freeze: "
                  << freezeReasonToString(pending_reason) << std::endl;

        interlock.onControlEvent(ControlEvent::FREEZE_NOW, pending_reason);
        if (interlock.state() == InterlockState::FAULT) {
            motion_faulted = true;
        }
    });

    guardian.setOnClearFreezeCallback([&]() {
        interlock.onControlEvent(ControlEvent::ALLOW_MOTION);
        if (interlock.state() == InterlockState::FAULT) {
            motion_faulted = true;
            return;
        }

        if (!startDemoStepTimer()) {
            motion_faulted = true;
            return;
        }

        motion_gate_open = true;
        std::cout << "[DEMO] resume" << std::endl;
    });

    guardian.setOnStateChangeCallback([&](GuardianState from, GuardianState to) {
        std::cout << "[DEMO] guardian=" << guardian.stateToString(from)
                  << " -> " << guardian.stateToString(to) << std::endl;
    });

    auto announceSceneState = [&](bool safe_now) {
        if (scene_state_known && safe_now == scene_was_safe) {
            return;
        }

        scene_state_known = true;
        scene_was_safe = safe_now;
        if (safe_now) {
            std::cout << "[DEMO] "
                      << (demo_armed ? "safe again" : "safe and ready to arm")
                      << std::endl;
        } else {
            std::cout << "[DEMO] scene unsafe" << std::endl;
        }
    };

    auto updateWaitingState = [&]() {
        const bool should_wait_for_ack =
            scene_is_safe && guardian.getState() == GuardianState::FROZEN_UNSAFE;

        if (should_wait_for_ack && !waiting_for_ack_announced) {
            std::cout << "[DEMO] waiting for ACK" << std::endl;
            waiting_for_ack_announced = true;
        }

        waiting_for_ack = should_wait_for_ack;
        if (!should_wait_for_ack) {
            waiting_for_ack_announced = false;
        }
    };

    auto maybeAdvanceDemoPose = [&]() -> bool {
        if (!demo_armed || !motion_gate_open || waiting_for_ack) {
            return true;
        }

        if (!demo_step_due.exchange(false, std::memory_order_relaxed)) {
            return true;
        }

        const DemoPoseStep& step = kDemoSequence[next_step_index];
        if (!stagePose(step)) {
            return false;
        }

        next_step_index = (next_step_index + 1) % kDemoSequence.size();
        return true;
    };

    cv::namedWindow(kDemoWindowName, cv::WINDOW_AUTOSIZE);

    FrameEvent frame_event;
    int consecutive_capture_failures = 0;

    while (true) {
        if (interlock.state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[DEMO] interlock fault (motion controller: "
                      << motion_controller_.lastErrorString() << ")" << std::endl;
            break;
        }

        if (!camera_capture.waitForNextFrame(frame_event)) {
            ++consecutive_capture_failures;
            std::cerr << "[DEMO] frame capture failed ("
                      << consecutive_capture_failures
                      << "/30). Retrying..." << std::endl;

            if (consecutive_capture_failures >= 30) {
                break;
            }

            std::string timer_error;
            if (!waitForRealtimeDelay(kCaptureRetryBackoff, timer_error)) {
                motion_faulted = true;
                std::cerr << "[DEMO] retry timer failed: " << timer_error
                          << std::endl;
                break;
            }
            continue;
        }

        consecutive_capture_failures = 0;
        processed_any_frame = true;

        const auto now = std::chrono::steady_clock::now();
        const SafetyResult result =
            vision_processor.process(frame_event.image_data, now);

        current_vision_state = result.state;
        scene_is_safe = (current_vision_state == SafetyState::SAFE);
        if (!scene_is_safe) {
            pending_reason = mapSafetyToFreezeReason(current_vision_state);
        }

        const GuardianState guardian_state_before_update = guardian.getState();
        if (demo_armed &&
            (guardian_state_before_update == GuardianState::SAFE_MONITORING ||
             guardian_state_before_update == GuardianState::RESET_PENDING)) {
            guardian.processFrame(scene_is_safe ? FrameStatus::FRAME_GOOD
                                                : FrameStatus::FRAME_BAD);
        }
        announceSceneState(scene_is_safe);

        if (scene_is_safe && !home_pose_staged) {
            if (!stagePose(kDemoHomeStep)) {
                motion_faulted = true;
                break;
            }
            home_pose_staged = true;
        }

        updateWaitingState();

        PhysicalButtonEvent button_event;
        while (button_module.poll(button_event)) {
            if (!requestFromButton(button_event)) {
                motion_faulted = true;
                break;
            }
        }

        if (motion_faulted) {
            break;
        }

        if (!maybeAdvanceDemoPose()) {
            motion_faulted = true;
            break;
        }

        if (interlock.state() == InterlockState::FAULT) {
            motion_faulted = true;
            std::cerr << "[DEMO] motion control fault: "
                      << motion_controller_.lastErrorString() << std::endl;
            break;
        }

        cv::Mat display_frame = frame_event.image_data.clone();
        const std::string guardian_state_text = guardian.getCurrentStateString();
        const std::string interlock_state_text =
            interlockStateToString(interlock.state());
        const std::string freeze_reason_text =
            freezeReasonToString(interlock.freezeReason());

        std::string demo_state_text = "WAIT_MARKER";
        if (!demo_armed) {
            demo_state_text = scene_is_safe ? "READY_TO_ARM" : "WAIT_SAFE";
        } else if (motion_gate_open) {
            demo_state_text = "RUNNING";
        } else if (waiting_for_ack) {
            demo_state_text = "WAIT_ACK";
        } else if (guardian.getState() == GuardianState::FROZEN_UNSAFE) {
            demo_state_text = "FROZEN";
        } else if (scene_is_safe) {
            demo_state_text = "SAFE_READY";
        }

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
                        (demo_armed ? "ARMED (r/button=ack)"
                                    : "DISARMED (a/button=arm)") +
                        " | READY_TO_ARM: " + (scene_is_safe ? "YES" : "NO"),
                    cv::Point(16, 120),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    cv::Scalar(255, 255, 0),
                    2);

        cv::putText(display_frame,
                    std::string("POSE: ") + current_pose_name,
                    cv::Point(16, 150),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 255, 255),
                    2);

        cv::putText(display_frame,
                    std::string("DEMO: ") + demo_state_text,
                    cv::Point(16, 180),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 255, 255),
                    2);

        cv::putText(display_frame,
                    std::string("FREEZE: ") + freeze_reason_text,
                    cv::Point(16, 210),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 255, 255),
                    2);

        cv::imshow(kDemoWindowName, display_frame);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27) {
            std::cout << "[DEMO] quit requested" << std::endl;
            break;
        }
        if (key == 'a' || key == 'A') {
            if (!requestContinue()) {
                motion_faulted = true;
                break;
            }
        }
        if (key == 'r' || key == 'R') {
            if (!requestContinue()) {
                motion_faulted = true;
                break;
            }
        }
    }

    cv::destroyWindow(kDemoWindowName);
    stopDemoStepTimer();
    motion_controller_.shutdown();

    if (!processed_any_frame) {
        std::cerr << "[DEMO] no frames processed. Check camera availability."
                  << std::endl;
        return 1;
    }

    if (motion_faulted) {
        return 1;
    }

    std::cerr << "[DEMO] frame stream ended." << std::endl;
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
    const char* button_module_status =
        button_module.available() ? button_module.statusString()
                                  : button_module.lastErrorString();
    if (button_module_status != nullptr &&
        std::string(button_module_status) != "no error" &&
        std::string(button_module_status) != "no status") {
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

            std::string timer_error;
            if (!waitForRealtimeDelay(kCaptureRetryBackoff, timer_error)) {
                motion_faulted = true;
                std::cerr << "[LIVE_TEST] retry timer failed: " << timer_error
                          << std::endl;
                break;
            }
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
