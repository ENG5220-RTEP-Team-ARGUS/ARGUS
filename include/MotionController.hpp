#pragma once

#include <cstdint>

enum class MotionControllerError {
    NONE,
    INVALID_ARGUMENT,
    NOT_INITIALISED,
    DEVICE_OPEN_FAILED,
    DEVICE_SELECT_FAILED,
    DEVICE_IO_FAILED
};

enum class MotionOutputState {
    UNINITIALISED,
    DISABLED,
    ENABLED,
    FAULT
};

struct MotionChannelMap {
    std::uint8_t base{0};
    std::uint8_t lower{1};
    std::uint8_t upper{2};
    std::uint8_t gripper{3};
};

struct MeArmJointTargets {
    std::uint16_t base_ticks{0};
    std::uint16_t lower_ticks{0};
    std::uint16_t upper_ticks{0};
    std::uint16_t gripper_ticks{0};
};

class MotionController {
public:
    static constexpr std::uint8_t kServoCount = 4;
    static constexpr std::uint8_t kPca9685ChannelCount = 16;
    static constexpr std::uint16_t kMaxPulseTicks = 4095;

    MotionController() noexcept;
    ~MotionController() noexcept;

    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;
    MotionController(MotionController&&) = delete;
    MotionController& operator=(MotionController&&) = delete;

    bool initialise(const char* i2c_device_path = "/dev/i2c-1",
                    std::uint8_t device_address = 0x40,
                    float pwm_frequency_hz = 50.0f,
                    MotionChannelMap channel_map = {}) noexcept;

    bool setTargets(const MeArmJointTargets& targets) noexcept;

    void freeze() noexcept;
    bool enable() noexcept;
    void shutdown() noexcept;

    bool isInitialised() const noexcept;
    bool isEnabled() const noexcept;
    bool isFrozen() const noexcept;
    MotionOutputState outputState() const noexcept;
    MotionControllerError lastError() const noexcept;
    const char* lastErrorString() const noexcept;

private:
    bool configurePca9685(float pwm_frequency_hz) noexcept;
    bool writeTargetsToHardware(const MeArmJointTargets& targets) noexcept;
    bool writeChannelPulse(std::uint8_t channel, std::uint16_t pulse_ticks) noexcept;
    bool writeRegister(std::uint8_t reg, std::uint8_t value) noexcept;
    bool writeRegisters(std::uint8_t start_reg,
                        const std::uint8_t* values,
                        std::uint8_t value_count) noexcept;
    bool setAllOutputsEnabled(bool enabled) noexcept;
    bool hasValidChannelMap(const MotionChannelMap& channel_map) const noexcept;
    void setFault(MotionControllerError error) noexcept;

    int i2c_fd_;
    MotionChannelMap channel_map_;
    MeArmJointTargets cached_targets_;
    MotionOutputState output_state_;
    MotionControllerError last_error_;
    bool frozen_;
};
