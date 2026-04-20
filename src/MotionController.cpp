#include "MotionController.hpp"
#include "CppTimerStdFuncCallback.h"

#include <array>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fcntl.h>
#include <mutex>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {
constexpr unsigned long kI2cSlaveRequest = 0x0703;

constexpr std::uint8_t kMode1Register = 0x00;
constexpr std::uint8_t kMode2Register = 0x01;
constexpr std::uint8_t kLed0OnLowRegister = 0x06;
constexpr std::uint8_t kAllLedOnLowRegister = 0xFA;
constexpr std::uint8_t kPrescaleRegister = 0xFE;

constexpr std::uint8_t kMode1SleepBit = 0x10;
constexpr std::uint8_t kMode1AutoIncrementBit = 0x20;
constexpr std::uint8_t kMode2TotemPoleBit = 0x04;
constexpr std::uint8_t kFullOffBit = 0x10;

constexpr float kPca9685OscillatorHz = 25000000.0f;
constexpr std::uint8_t kMinimumPrescale = 3;
constexpr std::uint8_t kMaximumPrescale = 255;
constexpr useconds_t kWakeDelayUs = 5000;

bool waitForWakeDelay(useconds_t delay_us) noexcept {
    if (delay_us == 0U) {
        return true;
    }

    std::mutex mutex;
    std::condition_variable condition;
    bool fired = false;

    CppTimerCallback timer;
    timer.registerEventCallback([&]() {
        std::lock_guard<std::mutex> lock(mutex);
        fired = true;
        condition.notify_one();
    });

    const std::uint64_t delay_ns =
        static_cast<std::uint64_t>(delay_us) * 1000ULL;
    try {
        timer.startns(static_cast<long>(delay_ns), ONESHOT);
    } catch (...) {
        return false;
    }

    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&]() { return fired; });
    timer.stop();
    return true;
}
}  // namespace

MotionController::MotionController() noexcept
    : i2c_fd_(-1),
      channel_map_(),
      cached_targets_(),
      output_state_(MotionOutputState::UNINITIALISED),
      last_error_(MotionControllerError::NONE),
      frozen_(true) {}

MotionController::~MotionController() noexcept {
    shutdown();
}

bool MotionController::initialise(const char* i2c_device_path,
                                  std::uint8_t device_address,
                                  float pwm_frequency_hz,
                                  MotionChannelMap channel_map) noexcept {
    shutdown();

    if (i2c_device_path == nullptr || !hasValidChannelMap(channel_map) ||
        pwm_frequency_hz <= 0.0f) {
        output_state_ = MotionOutputState::FAULT;
        last_error_ = MotionControllerError::INVALID_ARGUMENT;
        frozen_ = true;
        return false;
    }

    i2c_fd_ = ::open(i2c_device_path, O_RDWR | O_CLOEXEC);
    if (i2c_fd_ < 0) {
        output_state_ = MotionOutputState::FAULT;
        last_error_ = MotionControllerError::DEVICE_OPEN_FAILED;
        frozen_ = true;
        return false;
    }

    if (::ioctl(i2c_fd_,
                kI2cSlaveRequest,
                static_cast<unsigned long>(device_address)) < 0) {
        setFault(MotionControllerError::DEVICE_SELECT_FAILED);
        ::close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    channel_map_ = channel_map;

    if (!configurePca9685(pwm_frequency_hz)) {
        if (last_error_ == MotionControllerError::INVALID_ARGUMENT) {
            setFault(MotionControllerError::INVALID_ARGUMENT);
        } else {
            setFault(MotionControllerError::DEVICE_IO_FAILED);
        }
        ::close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    if (!setAllOutputsEnabled(false)) {
        setFault(MotionControllerError::DEVICE_IO_FAILED);
        ::close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    cached_targets_ = {};
    output_state_ = MotionOutputState::DISABLED;
    last_error_ = MotionControllerError::NONE;
    frozen_ = true;
    return true;
}

bool MotionController::setTargets(const MeArmJointTargets& targets) noexcept {
    if (output_state_ == MotionOutputState::UNINITIALISED) {
        last_error_ = MotionControllerError::NOT_INITIALISED;
        return false;
    }

    if (output_state_ == MotionOutputState::FAULT) {
        return false;
    }

    if (targets.base_ticks > kMaxPulseTicks ||
        targets.lower_ticks > kMaxPulseTicks ||
        targets.upper_ticks > kMaxPulseTicks ||
        targets.gripper_ticks > kMaxPulseTicks) {
        last_error_ = MotionControllerError::INVALID_ARGUMENT;
        return false;
    }

    cached_targets_ = targets;

    if (frozen_ || output_state_ != MotionOutputState::ENABLED) {
        last_error_ = MotionControllerError::NONE;
        return true;
    }

    if (!writeTargetsToHardware(cached_targets_)) {
        setFault(MotionControllerError::DEVICE_IO_FAILED);
        return false;
    }

    last_error_ = MotionControllerError::NONE;
    return true;
}

void MotionController::freeze() noexcept {
    frozen_ = true;

    if (i2c_fd_ < 0) {
        return;
    }

    if (!setAllOutputsEnabled(false)) {
        setFault(MotionControllerError::DEVICE_IO_FAILED);
        return;
    }

    if (output_state_ != MotionOutputState::FAULT) {
        output_state_ = MotionOutputState::DISABLED;
        last_error_ = MotionControllerError::NONE;
    }
}

bool MotionController::enable() noexcept {
    if (output_state_ == MotionOutputState::UNINITIALISED) {
        last_error_ = MotionControllerError::NOT_INITIALISED;
        return false;
    }

    if (output_state_ == MotionOutputState::FAULT) {
        return false;
    }

    if (!setAllOutputsEnabled(true) || !writeTargetsToHardware(cached_targets_)) {
        setFault(MotionControllerError::DEVICE_IO_FAILED);
        return false;
    }

    frozen_ = false;
    output_state_ = MotionOutputState::ENABLED;
    last_error_ = MotionControllerError::NONE;
    return true;
}

void MotionController::shutdown() noexcept {
    if (i2c_fd_ >= 0) {
        (void)setAllOutputsEnabled(false);
        ::close(i2c_fd_);
        i2c_fd_ = -1;
    }

    cached_targets_ = {};
    output_state_ = MotionOutputState::UNINITIALISED;
    last_error_ = MotionControllerError::NONE;
    frozen_ = true;
}

bool MotionController::isInitialised() const noexcept {
    return output_state_ != MotionOutputState::UNINITIALISED &&
           output_state_ != MotionOutputState::FAULT;
}

bool MotionController::isEnabled() const noexcept {
    return output_state_ == MotionOutputState::ENABLED;
}

bool MotionController::isFrozen() const noexcept {
    return frozen_;
}

MotionOutputState MotionController::outputState() const noexcept {
    return output_state_;
}

MotionControllerError MotionController::lastError() const noexcept {
    return last_error_;
}

const char* MotionController::lastErrorString() const noexcept {
    switch (last_error_) {
        case MotionControllerError::NONE:
            return "no error";
        case MotionControllerError::INVALID_ARGUMENT:
            return "invalid argument";
        case MotionControllerError::NOT_INITIALISED:
            return "controller not initialised";
        case MotionControllerError::DEVICE_OPEN_FAILED:
            return "failed to open I2C device";
        case MotionControllerError::DEVICE_SELECT_FAILED:
            return "failed to select I2C slave address";
        case MotionControllerError::DEVICE_IO_FAILED:
            return "PCA9685 I/O failed";
        default:
            return "unknown error";
    }
}

bool MotionController::configurePca9685(float pwm_frequency_hz) noexcept {
    const float prescale_value =
        (kPca9685OscillatorHz / (4096.0f * pwm_frequency_hz)) - 1.0f;
    const long prescale_long = std::lround(prescale_value);

    if (prescale_long < kMinimumPrescale || prescale_long > kMaximumPrescale) {
        last_error_ = MotionControllerError::INVALID_ARGUMENT;
        return false;
    }

    if (!writeRegister(kMode1Register, kMode1SleepBit)) {
        return false;
    }

    if (!writeRegister(kPrescaleRegister,
                       static_cast<std::uint8_t>(prescale_long))) {
        return false;
    }

    if (!writeRegister(kMode2Register, kMode2TotemPoleBit)) {
        return false;
    }

    if (!writeRegister(kMode1Register, kMode1AutoIncrementBit)) {
        return false;
    }

    if (!waitForWakeDelay(kWakeDelayUs)) {
        return false;
    }

    return true;
}

bool MotionController::writeTargetsToHardware(
    const MeArmJointTargets& targets) noexcept {
    return writeChannelPulse(channel_map_.base, targets.base_ticks) &&
           writeChannelPulse(channel_map_.lower, targets.lower_ticks) &&
           writeChannelPulse(channel_map_.upper, targets.upper_ticks) &&
           writeChannelPulse(channel_map_.gripper, targets.gripper_ticks);
}

bool MotionController::writeChannelPulse(std::uint8_t channel,
                                         std::uint16_t pulse_ticks) noexcept {
    if (channel >= kPca9685ChannelCount || pulse_ticks > kMaxPulseTicks) {
        last_error_ = MotionControllerError::INVALID_ARGUMENT;
        return false;
    }

    const std::uint8_t register_address = static_cast<std::uint8_t>(
        kLed0OnLowRegister + (4U * channel));
    const std::uint8_t payload[4] = {
        0x00,
        0x00,
        static_cast<std::uint8_t>(pulse_ticks & 0xFFU),
        static_cast<std::uint8_t>((pulse_ticks >> 8U) & 0x0FU)};

    return writeRegisters(register_address, payload, 4);
}

bool MotionController::writeRegister(std::uint8_t reg,
                                     std::uint8_t value) noexcept {
    return writeRegisters(reg, &value, 1);
}

bool MotionController::writeRegisters(std::uint8_t start_reg,
                                      const std::uint8_t* values,
                                      std::uint8_t value_count) noexcept {
    if (i2c_fd_ < 0 || values == nullptr || value_count == 0 || value_count > 4) {
        last_error_ = MotionControllerError::INVALID_ARGUMENT;
        return false;
    }

    std::uint8_t buffer[5] = {};
    buffer[0] = start_reg;

    for (std::uint8_t index = 0; index < value_count; ++index) {
        buffer[index + 1] = values[index];
    }

    const std::size_t bytes_to_write =
        static_cast<std::size_t>(value_count) + 1U;
    const ssize_t bytes_written =
        ::write(i2c_fd_, buffer, bytes_to_write);

    return bytes_written == static_cast<ssize_t>(bytes_to_write);
}

bool MotionController::setAllOutputsEnabled(bool enabled) noexcept {
    // Uses the PCA9685 full-off bit as the fast software inhibit path.
    const std::uint8_t payload[4] = {
        0x00,
        0x00,
        0x00,
        static_cast<std::uint8_t>(enabled ? 0x00 : kFullOffBit)};

    return writeRegisters(kAllLedOnLowRegister, payload, 4);
}

bool MotionController::hasValidChannelMap(
    const MotionChannelMap& channel_map) const noexcept {
    const std::array<std::uint8_t, kServoCount> channels = {
        channel_map.base,
        channel_map.lower,
        channel_map.upper,
        channel_map.gripper};

    for (std::size_t i = 0; i < channels.size(); ++i) {
        if (channels[i] >= kPca9685ChannelCount) {
            return false;
        }

        for (std::size_t j = i + 1; j < channels.size(); ++j) {
            if (channels[i] == channels[j]) {
                return false;
            }
        }
    }

    return true;
}

void MotionController::setFault(MotionControllerError error) noexcept {
    last_error_ = error;
    frozen_ = true;
    output_state_ = MotionOutputState::FAULT;

    if (i2c_fd_ >= 0) {
        (void)setAllOutputsEnabled(false);
    }
}
