/**
 * @file PhysicalButtonModule.hpp
 * @brief Debounced physical operator-button input module.
 */

#pragma once

#include <chrono>
#include <optional>
#include <string>

/**
 * @brief Semantic operator requests produced by button input.
 *
 * These are debounced press-edge semantic events, not raw electrical
 * transitions. AppController still routes these requests through the
 * guardian/interlock safety flow.
 */
enum class PhysicalButtonEvent {
    ARM_REQUEST,  ///< Request to arm/start controlled motion.
    DISARM_REQUEST,  ///< Request to disarm/stop controlled motion.
    ACK_REQUEST  ///< Request acknowledge/continue after freeze.
};

/**
 * @brief Configuration for physical button inputs.
 */
struct PhysicalButtonConfig {
    std::optional<int> arm_gpio;  ///< BCM GPIO for arm request, if present.
    std::optional<int> disarm_gpio;  ///< BCM GPIO for disarm request, if present.
    std::optional<int> acknowledge_gpio;  ///< BCM GPIO for acknowledge request, if present.
    bool active_low = true;  ///< Interpret low level as pressed when true.
    std::chrono::milliseconds debounce{50};  ///< Debounce interval.
};

/**
 * @brief GPIO character-device based physical button module.
 */
class PhysicalButtonModule {
public:
    /**
     * @brief Construct module from environment-derived configuration.
     */
    PhysicalButtonModule();

    /**
     * @brief Destroy module and release any open line descriptors.
     */
    ~PhysicalButtonModule();

    /**
     * @brief Construct module with explicit configuration.
     * @param config Button channel mapping and debounce options.
     */
    explicit PhysicalButtonModule(const PhysicalButtonConfig& config);

    /**
     * @brief Check whether at least one input channel is available.
     * @return `true` when module has at least one active channel.
     */
    bool available() const noexcept;

    /**
     * @brief Poll channels and emit a debounced semantic event if available.
     * @param event Output event populated on success.
     * @return `true` when an event is emitted; otherwise `false`.
     */
    bool poll(PhysicalButtonEvent& event) noexcept;

    /**
     * @brief Wait for next debounced event using blocking GPIO edge reads.
     * @param event Output event populated on success.
     * @param timeout Maximum wait duration.
     * @return `true` when event is received before timeout, otherwise `false`.
     */
    bool waitForEvent(PhysicalButtonEvent& event,
                      std::chrono::milliseconds timeout) noexcept;

    /**
     * @brief Read current acknowledge-button pressed state.
     * @param pressed Output flag populated on success.
     * @return `true` when read succeeded, otherwise `false`.
     */
    bool readAcknowledgePressed(bool& pressed) noexcept;

    /**
     * @brief Get last error string.
     * @return Null-terminated string with latest module error.
     */
    const char* lastErrorString() const noexcept;

    /**
     * @brief Get compact status summary string.
     * @return Null-terminated status string.
     */
    const char* statusString() const noexcept;

    /**
     * @brief Build button configuration from environment variables.
     * @return Parsed configuration.
     */
    static PhysicalButtonConfig configFromEnvironment() noexcept;

    /**
     * @brief Convert semantic event enum to readable string.
     * @param event Event value.
     * @return Null-terminated event label.
     */
    static const char* eventToString(PhysicalButtonEvent event) noexcept;

private:
    struct ChannelState {
        std::optional<int> gpio;
        PhysicalButtonEvent event;
        std::string value_path;
        std::string device_path;
        int line_fd = -1;
        bool active = false;
        bool initialised = false;
        bool last_sample_pressed = false;
        bool stable_pressed = false;
        std::chrono::steady_clock::time_point last_transition{};
    };

    PhysicalButtonConfig config_;
    ChannelState arm_channel_;
    ChannelState disarm_channel_;
    ChannelState acknowledge_channel_;
    bool available_ = false;
    std::string last_error_;
    std::string status_string_;

    static std::optional<int> parseEnvInt(const char* name) noexcept;
    static bool parseEnvBool(const char* name, bool default_value) noexcept;
    static std::chrono::milliseconds parseEnvDuration(
        const char* name,
        std::chrono::milliseconds default_value) noexcept;
    static std::string makeValuePath(int gpio);

    void initialiseChannel(ChannelState& channel,
                           std::optional<int> gpio,
                           PhysicalButtonEvent event) noexcept;
    bool sampleChannel(ChannelState& channel,
                       std::chrono::steady_clock::time_point now,
                       PhysicalButtonEvent& event) noexcept;
    bool readPressedState(const ChannelState& channel, bool& pressed) noexcept;
};
