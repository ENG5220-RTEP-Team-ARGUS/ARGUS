/**
 * @file MotionController.hpp
 * @brief PCA9685-based servo output controller for ARGUS.
 */

#pragma once

#include <cstdint>

/**
 * @brief Error codes reported by MotionController operations.
 */
enum class MotionControllerError {
    NONE,  ///< No error.
    INVALID_ARGUMENT,  ///< Invalid input argument.
    NOT_INITIALISED,  ///< Controller is not initialised.
    DEVICE_OPEN_FAILED,  ///< Failed to open I2C device.
    DEVICE_SELECT_FAILED,  ///< Failed to select I2C slave address.
    DEVICE_IO_FAILED  ///< I/O transaction failed.
};

/**
 * @brief High-level state of motion output availability.
 */
enum class MotionOutputState {
    UNINITIALISED,  ///< Hardware not initialised.
    DISABLED,  ///< Initialised but outputs disabled.
    ENABLED,  ///< Outputs enabled and can accept target writes.
    FAULT  ///< Fault state; output path blocked until reset/re-init.
};

/**
 * @brief Mapping of logical MeArm joints to PCA9685 channels.
 */
struct MotionChannelMap {
    std::uint8_t base{0};  ///< Base joint PWM channel.
    std::uint8_t lower{4};  ///< Lower joint PWM channel.
    std::uint8_t upper{8};  ///< Upper joint PWM channel.
    std::uint8_t gripper{12};  ///< Gripper joint PWM channel.
};

/**
 * @brief Raw PWM tick targets for each MeArm joint.
 */
struct MeArmJointTargets {
    std::uint16_t base_ticks{0};  ///< Base target pulse ticks.
    std::uint16_t lower_ticks{0};  ///< Lower target pulse ticks.
    std::uint16_t upper_ticks{0};  ///< Upper target pulse ticks.
    std::uint16_t gripper_ticks{0};  ///< Gripper target pulse ticks.
};

/**
 * @brief Low-level motion output driver for the PCA9685 servo board.
 */
class MotionController {
public:
    static constexpr std::uint8_t kServoCount = 4;  ///< Number of logical servos controlled.
    static constexpr std::uint8_t kPca9685ChannelCount = 16;  ///< PCA9685 total channels.
    static constexpr std::uint16_t kMaxPulseTicks = 4095;  ///< Maximum 12-bit pulse value.

    /**
     * @brief Construct motion controller in uninitialised state.
     */
    MotionController() noexcept;

    /**
     * @brief Destroy motion controller and release resources.
     */
    ~MotionController() noexcept;

    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;
    MotionController(MotionController&&) = delete;
    MotionController& operator=(MotionController&&) = delete;

    /**
     * @brief Initialise PCA9685 output path.
     * @param i2c_device_path I2C device path (for example `/dev/i2c-1`).
     * @param device_address I2C slave address of the PCA9685.
     * @param pwm_frequency_hz PWM frequency in Hz.
     * @param channel_map Logical joint-to-channel mapping.
     * @return `true` on success, otherwise `false` and lastError() set.
     */
    bool initialise(
        const char* i2c_device_path = "/dev/i2c-1",
        std::uint8_t device_address = 0x40,
        float pwm_frequency_hz = 50.0f,
        MotionChannelMap channel_map = {}) noexcept;

    /**
     * @brief Write joint targets to hardware when motion is enabled.
     * @param targets Joint pulse-tick targets.
     * @return `true` on success, otherwise `false`.
     */
    bool setTargets(const MeArmJointTargets& targets) noexcept;

    /**
     * @brief Freeze/disable motion output immediately.
     */
    void freeze() noexcept;

    /**
     * @brief Enable motion output after successful initialisation.
     * @return `true` when enable succeeds, otherwise `false`.
     */
    bool enable() noexcept;

    /**
     * @brief Disable outputs and close hardware resources.
     */
    void shutdown() noexcept;

    /**
     * @brief Check whether controller was successfully initialised.
     * @return `true` if initialised, otherwise `false`.
     */
    bool isInitialised() const noexcept;

    /**
     * @brief Check whether motion output is currently enabled.
     * @return `true` if enabled, otherwise `false`.
     */
    bool isEnabled() const noexcept;

    /**
     * @brief Check whether output is currently frozen.
     * @return `true` if frozen, otherwise `false`.
     */
    bool isFrozen() const noexcept;

    /**
     * @brief Get current output state.
     * @return Output state enum.
     */
    MotionOutputState outputState() const noexcept;

    /**
     * @brief Get last controller error code.
     * @return Error code.
     */
    MotionControllerError lastError() const noexcept;

    /**
     * @brief Get human-readable last error string.
     * @return Null-terminated string describing last error.
     */
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
