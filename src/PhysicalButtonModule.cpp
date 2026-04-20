#include "PhysicalButtonModule.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <limits>
#include <string>
#include <vector>

#include <linux/gpio.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

constexpr int kDefaultAckGpio = 24;
constexpr const char* kButtonConsumer = "ARGUS_BUTTON";

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

std::string toLowerCopy(std::string text) {
    for (char& ch : text) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

bool containsInsensitive(const std::string& haystack, const char* needle) {
    if (needle == nullptr || *needle == '\0') {
        return false;
    }

    const std::string haystack_lower = toLowerCopy(haystack);
    const std::string needle_lower = toLowerCopy(needle);
    return haystack_lower.find(needle_lower) != std::string::npos;
}

std::string boundedCString(const char* text, std::size_t max_length) {
    if (text == nullptr) {
        return {};
    }

    return std::string(text, ::strnlen(text, max_length));
}

bool equalsInsensitive(const std::string& lhs, const std::string& rhs) {
    return toLowerCopy(lhs) == toLowerCopy(rhs);
}

struct GpioChipCandidate {
    std::string path;
    std::string name;
    std::string label;
    unsigned int lines = 0;
    int preference = 1;
};

std::vector<std::string> buildLineNameCandidates(int gpio) {
    return {
        "GPIO" + std::to_string(gpio),
        "gpio" + std::to_string(gpio),
    };
}

int chipPreference(const GpioChipCandidate& candidate) {
    if (containsInsensitive(candidate.label, "pinctrl") ||
        containsInsensitive(candidate.label, "rp1") ||
        containsInsensitive(candidate.name, "pinctrl") ||
        containsInsensitive(candidate.name, "rp1") ||
        containsInsensitive(candidate.name, "bcm")) {
        return 0;
    }
    return 1;
}

std::vector<GpioChipCandidate> enumerateGpioChips() {
    std::vector<GpioChipCandidate> candidates;

    DIR* dir = ::opendir("/dev");
    if (dir == nullptr) {
        return candidates;
    }

    while (dirent* entry = ::readdir(dir)) {
        if (std::strncmp(entry->d_name, "gpiochip", 8) != 0) {
            continue;
        }

        std::string path = "/dev/";
        path += entry->d_name;

        int chip_fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
        if (chip_fd < 0) {
            continue;
        }

        gpiochip_info info{};
        if (::ioctl(chip_fd, GPIO_GET_CHIPINFO_IOCTL, &info) == 0) {
            GpioChipCandidate candidate;
            candidate.path = path;
            candidate.name = boundedCString(info.name, sizeof(info.name));
            candidate.label = boundedCString(info.label, sizeof(info.label));
            candidate.lines = info.lines;
            candidate.preference = chipPreference(candidate);
            candidates.push_back(candidate);
        }

        ::close(chip_fd);
    }

    ::closedir(dir);

    std::stable_sort(candidates.begin(),
                     candidates.end(),
                     [](const GpioChipCandidate& lhs, const GpioChipCandidate& rhs) {
                         if (lhs.preference != rhs.preference) {
                             return lhs.preference < rhs.preference;
                         }
                         return lhs.path < rhs.path;
                     });
    return candidates;
}

std::uint64_t makePreferredLineFlags(bool active_low) {
    std::uint64_t flags = static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_INPUT) |
                          static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_EDGE_RISING) |
                          static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_EDGE_FALLING);
    if (active_low) {
        flags |= static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_ACTIVE_LOW);
        flags |= static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_BIAS_PULL_UP);
    } else {
        flags |= static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN);
    }
    return flags;
}

std::uint64_t makeFallbackLineFlags(bool active_low) {
    std::uint64_t flags = static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_INPUT) |
                          static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_EDGE_RISING) |
                          static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_EDGE_FALLING);
    if (active_low) {
        flags |= static_cast<std::uint64_t>(GPIO_V2_LINE_FLAG_ACTIVE_LOW);
    }
    return flags;
}

bool requestLineFromChip(const std::string& chip_path,
                         int line_offset,
                         std::uint64_t flags,
                         int& line_fd,
                         std::string& error_text) noexcept {
    line_fd = -1;

    int chip_fd = ::open(chip_path.c_str(), O_RDWR | O_CLOEXEC);
    if (chip_fd < 0) {
        error_text = chip_path + ": open failed: " + std::strerror(errno);
        return false;
    }

    gpio_v2_line_request request{};
    request.offsets[0] = static_cast<std::uint32_t>(line_offset);
    request.num_lines = 1;
    request.config.flags = flags;
    request.config.num_attrs = 0;
    std::memset(request.consumer, 0, sizeof(request.consumer));
    std::strncpy(request.consumer, kButtonConsumer, sizeof(request.consumer) - 1);

    if (::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) == 0 &&
        request.fd >= 0) {
        line_fd = request.fd;
        ::close(chip_fd);
        return true;
    }

    const int saved_errno = errno;
    error_text = chip_path + " line " + std::to_string(line_offset) +
                 ": " + std::strerror(saved_errno);
    ::close(chip_fd);
    return false;
}

std::optional<int> findNamedLineOffset(const GpioChipCandidate& chip, int gpio) {
    const std::vector<std::string> target_names = buildLineNameCandidates(gpio);

    int chip_fd = ::open(chip.path.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd < 0) {
        return std::nullopt;
    }

    std::optional<int> resolved_offset;
    for (unsigned int offset = 0; offset < chip.lines; ++offset) {
        gpio_v2_line_info info{};
        info.offset = offset;
        if (::ioctl(chip_fd, GPIO_V2_GET_LINEINFO_IOCTL, &info) != 0) {
            continue;
        }

        const std::string line_name = boundedCString(info.name, sizeof(info.name));
        for (const std::string& target_name : target_names) {
            if (equalsInsensitive(line_name, target_name)) {
                resolved_offset = static_cast<int>(offset);
                break;
            }
        }

        if (resolved_offset.has_value()) {
            break;
        }
    }

    ::close(chip_fd);
    return resolved_offset;
}

bool requestButtonLine(int gpio,
                       bool active_low,
                       std::string& device_path,
                       int& line_fd,
                       std::string& error_text) {
    const std::vector<GpioChipCandidate> chips = enumerateGpioChips();
    if (chips.empty()) {
        error_text = "no /dev/gpiochip* devices found";
        return false;
    }

    const std::uint64_t preferred_flags = makePreferredLineFlags(active_low);
    const std::uint64_t fallback_flags = makeFallbackLineFlags(active_low);

    auto tryResolvedRequest = [&](const GpioChipCandidate& chip,
                                  int requested_offset,
                                  const char* resolution_label) {
        std::string attempt_error;
        if (requestLineFromChip(chip.path,
                                requested_offset,
                                preferred_flags,
                                line_fd,
                                attempt_error) ||
            requestLineFromChip(chip.path,
                                requested_offset,
                                fallback_flags,
                                line_fd,
                                attempt_error)) {
            device_path = chip.path + " " + resolution_label + " GPIO" +
                          std::to_string(gpio) + " (offset " +
                          std::to_string(requested_offset) + ")";
            error_text.clear();
            return true;
        }

        error_text = attempt_error;
        return false;
    };

    for (const GpioChipCandidate& chip : chips) {
        const std::optional<int> named_offset = findNamedLineOffset(chip, gpio);
        if (!named_offset.has_value()) {
            continue;
        }

        if (tryResolvedRequest(chip, *named_offset, "named")) {
            return true;
        }
    }

    for (const GpioChipCandidate& chip : chips) {
        if (gpio < 0 || static_cast<unsigned int>(gpio) >= chip.lines) {
            continue;
        }

        if (tryResolvedRequest(chip, gpio, "raw-offset")) {
            return true;
        }
    }

    if (error_text.empty()) {
        error_text = "unable to request GPIO" + std::to_string(gpio) +
                     " on any gpiochip device";
    }

    return false;
}

}  // namespace

PhysicalButtonModule::PhysicalButtonModule()
    : PhysicalButtonModule(configFromEnvironment()) {}

PhysicalButtonModule::~PhysicalButtonModule() {
    if (arm_channel_.line_fd >= 0) {
        ::close(arm_channel_.line_fd);
        arm_channel_.line_fd = -1;
    }
    if (disarm_channel_.line_fd >= 0) {
        ::close(disarm_channel_.line_fd);
        disarm_channel_.line_fd = -1;
    }
    if (acknowledge_channel_.line_fd >= 0) {
        ::close(acknowledge_channel_.line_fd);
        acknowledge_channel_.line_fd = -1;
    }
}

PhysicalButtonModule::PhysicalButtonModule(const PhysicalButtonConfig& config)
    : config_(config) {
    initialiseChannel(arm_channel_, config_.arm_gpio, PhysicalButtonEvent::ARM_REQUEST);
    initialiseChannel(disarm_channel_,
                      config_.disarm_gpio,
                      PhysicalButtonEvent::DISARM_REQUEST);
    initialiseChannel(acknowledge_channel_,
                      config_.acknowledge_gpio,
                      PhysicalButtonEvent::ACK_REQUEST);

    available_ = acknowledge_channel_.active;
    if (acknowledge_channel_.active) {
        status_string_ = acknowledge_channel_.device_path +
                         ", initial=" +
                         std::string(acknowledge_channel_.stable_pressed ? "PRESSED"
                                                                         : "RELEASED") +
                         ", debounce=" + std::to_string(config_.debounce.count()) +
                         "ms";
    }
    if (!available_ && last_error_.empty()) {
        last_error_ = "acknowledge button not configured";
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

bool PhysicalButtonModule::waitForEvent(PhysicalButtonEvent& event,
                                        std::chrono::milliseconds timeout) noexcept {
    std::array<ChannelState*, 3> channels{
        &disarm_channel_,
        &acknowledge_channel_,
        &arm_channel_,
    };
    std::vector<pollfd> fds;
    std::vector<ChannelState*> ready_channels;
    fds.reserve(channels.size());
    ready_channels.reserve(channels.size());

    for (ChannelState* channel : channels) {
        if (channel == nullptr || !channel->active || channel->line_fd < 0) {
            continue;
        }

        pollfd fd{};
        fd.fd = channel->line_fd;
        fd.events = POLLIN;
        fds.push_back(fd);
        ready_channels.push_back(channel);
    }

    if (fds.empty()) {
        return false;
    }

    int timeout_ms = -1;
    if (timeout.count() >= 0) {
        const long long requested_timeout = timeout.count();
        timeout_ms = static_cast<int>(
            std::min<long long>(requested_timeout, std::numeric_limits<int>::max()));
    }

    const int ready = ::poll(fds.data(), static_cast<nfds_t>(fds.size()), timeout_ms);
    if (ready <= 0) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < fds.size(); ++i) {
        if ((fds[i].revents & POLLIN) == 0) {
            continue;
        }

        ChannelState& channel = *ready_channels[i];
        gpio_v2_line_event line_event{};
        const ssize_t read_bytes = ::read(channel.line_fd, &line_event, sizeof(line_event));
        if (read_bytes != static_cast<ssize_t>(sizeof(line_event))) {
            continue;
        }

        bool pressed = false;
        if (line_event.id == GPIO_V2_LINE_EVENT_RISING_EDGE) {
            pressed = true;
        } else if (line_event.id == GPIO_V2_LINE_EVENT_FALLING_EDGE) {
            pressed = false;
        } else {
            continue;
        }

        if (pressed == channel.stable_pressed) {
            continue;
        }

        const auto since_last_transition = now - channel.last_transition;
        if (since_last_transition < config_.debounce) {
            continue;
        }

        channel.last_transition = now;
        channel.last_sample_pressed = pressed;
        channel.stable_pressed = pressed;
        if (pressed) {
            event = channel.event;
            return true;
        }
    }

    return false;
}

bool PhysicalButtonModule::readAcknowledgePressed(bool& pressed) noexcept {
    if (!acknowledge_channel_.active) {
        return false;
    }

    if (!readPressedState(acknowledge_channel_, pressed)) {
        last_error_ = std::string("unable to read ") + acknowledge_channel_.device_path +
                      " (" + acknowledge_channel_.value_path + ")";
        return false;
    }

    return true;
}

const char* PhysicalButtonModule::lastErrorString() const noexcept {
    if (last_error_.empty()) {
        return "no error";
    }
    return last_error_.c_str();
}

const char* PhysicalButtonModule::statusString() const noexcept {
    if (status_string_.empty()) {
        return "no status";
    }
    return status_string_.c_str();
}

PhysicalButtonConfig PhysicalButtonModule::configFromEnvironment() noexcept {
    PhysicalButtonConfig config;
    config.arm_gpio = parseEnvInt("ARGUS_BUTTON_ARM_GPIO");
    config.disarm_gpio = parseEnvInt("ARGUS_BUTTON_DISARM_GPIO");
    config.acknowledge_gpio =
        parseEnvInt("ARGUS_BUTTON_ACK_GPIO").value_or(kDefaultAckGpio);
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
    return "GPIO" + std::to_string(gpio);
}

void PhysicalButtonModule::initialiseChannel(ChannelState& channel,
                                             std::optional<int> gpio,
                                             PhysicalButtonEvent event) noexcept {
    if (channel.line_fd >= 0) {
        ::close(channel.line_fd);
        channel.line_fd = -1;
    }

    channel.gpio = gpio;
    channel.event = event;
    channel.value_path.clear();
    channel.device_path.clear();
    channel.active = false;
    channel.initialised = false;
    channel.last_sample_pressed = false;
    channel.stable_pressed = false;
    channel.last_transition = std::chrono::steady_clock::time_point{};

    if (!gpio.has_value()) {
        return;
    }

    channel.value_path = makeValuePath(*gpio);

    std::string error_text;
    if (!requestButtonLine(*gpio,
                           config_.active_low,
                           channel.device_path,
                           channel.line_fd,
                           error_text)) {
        if (event == PhysicalButtonEvent::ACK_REQUEST) {
            last_error_ = error_text.empty()
                              ? std::string("unable to request ") + channel.value_path
                              : error_text;
        }
        return;
    }

    bool pressed = false;
    if (!readPressedState(channel, pressed)) {
        if (event == PhysicalButtonEvent::ACK_REQUEST) {
            last_error_ = std::string("unable to read ") + channel.device_path +
                          " (" + channel.value_path + ")";
        }
        ::close(channel.line_fd);
        channel.line_fd = -1;
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
        if (channel.event == PhysicalButtonEvent::ACK_REQUEST) {
            last_error_ = std::string("unable to read ") + channel.device_path +
                          " (" + channel.value_path + ")";
        }
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
    if (channel.line_fd < 0) {
        return false;
    }

    gpio_v2_line_values values{};
    values.mask = 1;
    if (::ioctl(channel.line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) != 0) {
        return false;
    }

    pressed = (values.bits & 1ULL) != 0ULL;
    return true;
}
