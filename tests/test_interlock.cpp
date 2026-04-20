#include "RobotInterlock.hpp"

#include <cassert>

namespace {
class FakeHardware final : public RobotHardware {
public:
    bool freezeMotion() noexcept override {
        ++freeze_calls;
        return freeze_ok;
    }

    bool enableMotion() noexcept override {
        ++enable_calls;
        return enable_ok;
    }

    bool freeze_ok = true;
    bool enable_ok = true;
    int freeze_calls = 0;
    int enable_calls = 0;
};
}  // namespace

int main() {
    FakeHardware hardware;
    RobotInterlock interlock(hardware);

    assert(interlock.state() == InterlockState::SAFE);
    assert(interlock.motionAllowed());
    assert(interlock.freezeReason() == FreezeReason::NONE);

    interlock.onControlEvent(ControlEvent::FREEZE_NOW, FreezeReason::MARKER_LOST);
    assert(interlock.state() == InterlockState::FROZEN);
    assert(!interlock.motionAllowed());
    assert(interlock.freezeReason() == FreezeReason::MARKER_LOST);
    assert(hardware.freeze_calls == 1);

    interlock.operatorAcknowledge();
    interlock.onControlEvent(ControlEvent::ALLOW_MOTION);
    assert(interlock.state() == InterlockState::SAFE);
    assert(interlock.motionAllowed());
    assert(interlock.freezeReason() == FreezeReason::NONE);
    assert(hardware.enable_calls == 1);

    interlock.guardianHeartbeat(10);
    interlock.watchdogCheck(/*now=*/20, /*max_allowed_delay=*/5);
    assert(interlock.state() == InterlockState::FROZEN);
    assert(interlock.freezeReason() == FreezeReason::WATCHDOG_TIMEOUT);

    return 0;
}
