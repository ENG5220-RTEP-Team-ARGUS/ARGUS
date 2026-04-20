#include "GuardianStateMachine.hpp"

#include <cassert>

int main() {
    GuardianStateMachine guardian(/*freezeCount=*/2, /*recoverCount=*/2);

    bool freeze_callback_called = false;
    bool clear_callback_called = false;
    int transition_count = 0;
    guardian.setOnFreezeCallback([&]() { freeze_callback_called = true; });
    guardian.setOnClearFreezeCallback([&]() { clear_callback_called = true; });
    guardian.setOnStateChangeCallback(
        [&](GuardianState, GuardianState) { ++transition_count; });

    assert(guardian.getState() == GuardianState::SAFE_MONITORING);
    assert(guardian.isMotionAllowed());

    guardian.processFrame(FrameStatus::FRAME_BAD);
    assert(guardian.getState() == GuardianState::SAFE_MONITORING);
    assert(guardian.getBadCount() == 1);
    assert(!freeze_callback_called);

    guardian.processFrame(FrameStatus::FRAME_BAD);
    assert(guardian.getState() == GuardianState::FROZEN_UNSAFE);
    assert(guardian.isMotionBlocked());
    assert(freeze_callback_called);

    guardian.operatorAcknowledge();
    assert(guardian.getState() == GuardianState::RESET_PENDING);

    guardian.processFrame(FrameStatus::FRAME_GOOD);
    assert(guardian.getState() == GuardianState::RESET_PENDING);

    guardian.processFrame(FrameStatus::FRAME_GOOD);
    assert(guardian.getState() == GuardianState::SAFE_MONITORING);
    assert(guardian.isMotionAllowed());
    assert(clear_callback_called);
    assert(guardian.getBadCount() == 0);
    assert(guardian.getGoodCount() == 0);
    assert(transition_count >= 3);

    return 0;
}
