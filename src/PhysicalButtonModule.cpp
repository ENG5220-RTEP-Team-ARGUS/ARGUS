#include "PhysicalButtonModule.hpp"

#include <cerrno>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <limits>

namespace {

bool parseBoolText(const char* text, bool default_value) noexcept {
    if (text == nullptr) {
        return default_value;
    }

    std::string value(text);
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }

    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    return default_value;
}

}  // namespace

PhysicalButtonModule::PhysicalButtonModule()
    : PhysicalButtonModule(configFromEnvironment()) {}

PhysicalButtonModule::PhysicalButtonModule(const PhysicalButtonConfig& config)
    : config_(config) {
    initialiseChannel(arm_channel_, config_.arm_gpio, PhysicalButtonEvent::ARM_REQUEST);
    initialiseChannel(disarm_channel_,
                      config_.disarm_gpio,
                      PhysicalButtonEvent::DISARM_REQUEST);
    initialiseChannel(acknowledge_channel_,
                      config_.acknowledge_gpio,
                      PhysicalButtonEvent::ACK_REQUEST);

    available_ = arm_channel_.active || disarm_channel_.active || acknowledge_channel_.active;

    if (!available_ && last_error_.empty()) {
        last_error_ = "no physical buttons configured";
    }
}

bool PhysicalButtonModule::available() const noexcept {
    return available_;
}

bool PhysicalButtonModule::poll(PhysicalButtonEvent& event) noexcept {
    const auto now = std::chrono::steady_clock::now();

    return sampleChannel(disarm_channel_, now, event) ||
           sampleChannel(acknowledge_channel_, now, event) ||
           sampleChannel(arm_channel_, now, event);
}

const char* PhysicalButtonModule::lastErrorString() const noexcept {
    if (last_error_.empty()) {
        return "no error";
    }
    return last_error_.c_str();
}

PhysicalButtonConfig PhysicalButtonModule::configFromEnvironment() noexcept {
    PhysicalButtonConfig config;
    config.arm_gpio = parseEnvInt("ARGUS_BUTTON_ARM_GPIO");
    config.disarm_gpio = parseEnvInt("ARGUS_BUTTON_DISARM_GPIO");
    config.acknowledge_gpio = parseEnvInt("ARGUS_BUTTON_ACK_GPIO");
    config.active_low = parseEnvBool("ARGUS_BUTTON_ACTIVE_LOW", true);
    config.debounce =
        parseEnvDuration("ARGUS_BUTTON_DEBOUNCE_MS", std::chrono::milliseconds(50));
    return config;
}

const char* PhysicalButtonModule::eventToString(PhysicalButtonEvent event) noexcept {
    switch (event) {
        case PhysicalButtonEvent::ARM_REQUEST:
            return "ARM_REQUEST";
        case PhysicalButtonEvent::DISARM_REQUEST:
            return "DISARM_REQUEST";
        case PhysicalButtonEvent::ACK_REQUEST:
            return "ACK_REQUEST";
        default:
            return "UNKNOWN";
    }
}

std::optional<int> PhysicalButtonModule::parseEnvInt(const char* name) noexcept {
    const char* text = std::getenv(name);
    if (text == nullptr || *text == '\0') {
        return std::nullopt;
    }

    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || *end != '\0' ||
        value < 0 || value > std::numeric_limits<int>::max()) {
        return std::nullopt;
    }

    return static_cast<int>(value);
}

bool PhysicalButtonModule::parseEnvBool(const char* name, bool default_value) noexcept {
    return parseBoolText(std::getenv(name), default_value);
}

std::chrono::milliseconds PhysicalButtonModule::parseEnvDuration(
    const char* name,
    std::chrono::milliseconds default_value) noexcept {
    const auto value = parseEnvInt(name);
    if (!value.has_value()) {
        return default_value;
    }

    return std::chrono::milliseconds(*value);
}

std::string PhysicalButtonModule::makeValuePath(int gpio) {
    return "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value";
}

void PhysicalButtonModule::initialiseChannel(ChannelState& channel,
                                             std::optional<int> gpio,
                                             PhysicalButtonEvent event) noexcept {
    channel.gpio = gpio;
    channel.event = event;
    channel.value_path.clear();
    channel.active = false;
    channel.initialised = false;
    channel.last_sample_pressed = false;
    channel.stable_pressed = false;
    channel.last_transition = std::chrono::steady_clock::time_point{};

    if (!gpio.has_value()) {
        return;
    }

    channel.value_path = makeValuePath(*gpio);
    bool pressed = false;
    if (!readPressedState(channel, pressed)) {
        last_error_ = std::string("unable to read ") + channel.value_path;
        return;
    }

    channel.active = true;
    channel.initialised = true;
    channel.last_sample_pressed = pressed;
    channel.stable_pressed = pressed;
    channel.last_transition = std::chrono::steady_clock::now();
}

bool PhysicalButtonModule::sampleChannel(ChannelState& channel,
                                         std::chrono::steady_clock::time_point now,
                                         PhysicalButtonEvent& event) noexcept {
    if (!channel.active) {
        return false;
    }

    bool pressed = false;
    if (!readPressedState(channel, pressed)) {
        last_error_ = std::string("unable to read ") + channel.value_path;
        return false;
    }

    if (!channel.initialised) {
        channel.initialised = true;
        channel.last_sample_pressed = pressed;
        channel.stable_pressed = pressed;
        channel.last_transition = now;
        return false;
    }

    if (pressed != channel.last_sample_pressed) {
        channel.last_sample_pressed = pressed;
        channel.last_transition = now;
    }

    if (pressed == channel.stable_pressed) {
        return false;
    }

    const auto elapsed = now - channel.last_transition;
    if (elapsed < config_.debounce) {
        return false;
    }

    channel.stable_pressed = pressed;
    if (pressed) {
        event = channel.event;
        return true;
    }

    return false;
}

bool PhysicalButtonModule::readPressedState(const ChannelState& channel,
                                            bool& pressed) noexcept {
    std::ifstream input(channel.value_path);
    if (!input.is_open()) {
        return false;
    }

    int raw = -1;
    if (!(input >> raw)) {
        return false;
    }

    const bool logical_high = (raw != 0);
    pressed = config_.active_low ? !logical_high : logical_high;
    return true;
}
