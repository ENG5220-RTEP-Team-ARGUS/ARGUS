#pragma once

#include <chrono>
#include <optional>
#include <string>

// Semantic requests emitted by the physical operator button module.
//
// These are not hardware-level press/release notifications. They are
// debounced, press-edge events that tell AppController what the operator
// wants the system to do. AppController must still route the request through
// the existing guardian/interlock flow.
//
// Current wired button:
// BCM GPIO24 (physical pin 18), active-low input, GPIO character-device
// request with pull-up bias, software debounce.
enum class PhysicalButtonEvent {
    ARM_REQUEST,
    DISARM_REQUEST,
    ACK_REQUEST
};

struct PhysicalButtonConfig {
    std::optional<int> arm_gpio;
    std::optional<int> disarm_gpio;
    std::optional<int> acknowledge_gpio;
    bool active_low = true;
    std::chrono::milliseconds debounce{50};
};

class PhysicalButtonModule {
public:
    PhysicalButtonModule();
    ~PhysicalButtonModule();
    explicit PhysicalButtonModule(const PhysicalButtonConfig& config);

    bool available() const noexcept;
    bool poll(PhysicalButtonEvent& event) noexcept;
    const char* lastErrorString() const noexcept;
    const char* statusString() const noexcept;

    static PhysicalButtonConfig configFromEnvironment() noexcept;
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
