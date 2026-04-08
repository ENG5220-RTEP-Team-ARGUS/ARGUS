# ARGUS Compliance Matrix

## Purpose

This document turns the Bernd Porr reference projects in the local `compliance/`
folder into an explicit compliance plan for ARGUS. The goal is not to rewrite
ARGUS around those repos blindly. The goal is to adopt what is relevant,
document what is only guidance, and state clearly what is not applicable to the
current hardware/software architecture.

## Current ARGUS Baseline

ARGUS currently has a working Raspberry Pi hardware path:

- `AppController` orchestrates the application flow.
- `CameraCapture` provides live frames.
- `VisionProcessor` evaluates ArUco-based safety.
- `GuardianStateMachine` decides freeze/recovery state.
- `RobotInterlock` gates motion.
- `MotionController` drives a PCA9685-backed MeArm path.
- `PhysicalButtonModule` provides operator input on GPIO24.

The project is functional, but it is not yet aligned with the preferred
realtime/event-driven style shown in the Bernd Porr references.

## Matrix

| Reference repo | Role for ARGUS | Current fit | Main gap | Planned action | Priority |
| --- | --- | --- | --- | --- | --- |
| `realtime_cpp_coding` | Compliance standard and design checklist | Partial | ARGUS still has polling-heavy control loops and ad hoc timing | Use as the top-level standard for all compliance decisions and document gaps against it | High |
| `cppTimer` | Timer implementation source | High | Directly integrated, but button test still contains a small polling sleep | Vendor `cppTimer` under `third_party/` and use it as the timer primitive for control pacing | High |
| `libcamera2opencv` | Camera backend source | High | Bundled backend is integrated, but still needs Pi validation and dependency availability | Vendor `libcamera2opencv` under `third_party/`, try it first in auto mode, and fall back to OpenCV/V4L2 only if it fails | High |
| `cpp_event_callbacks` | Architectural guidance for event handoff | Medium | Top-level control still polls for button/camera/demo progression instead of receiving events | Refactor timer/frame/button flow toward callback-driven interfaces without redesigning the whole architecture | Medium |
| `rpi_pwm` | Alternative PWM path | Not applicable to current design | ARGUS uses PCA9685 over I2C, not Pi PWM GPIO18/19 | Document as out of scope unless motion hardware is redesigned | Low |

## File-Level Gap Assessment

### `src/AppController.cpp`

Current compliance gaps:

- uses `std::this_thread::sleep_for(...)` for:
  - motion-home settle timing
  - smoke-test dwell timing
  - full-demo pacing
  - retry pacing in interactive modes
- contains polling loops for:
  - button draining
  - demo/live progression

Compliance action:

- replace sleep-driven periodic behavior with vendored `cppTimer`
- move recurring events into explicit timer callbacks
- keep `AppController` as orchestrator, but reduce loop-centric control

### `src/CameraCapture.cpp` and `include/CameraCapture.hpp`

Current compliance gaps:

- camera acquisition is based on OpenCV `VideoCapture`
- Pi handling is pragmatic and working, but not aligned with the callback-first
  `libcamera2opencv` model

Compliance action:

- keep the `CameraCapture` role intact
- refactor implementation behind that interface
- host the vendored `libcamera2opencv` backend behind that boundary
- prefer the Bernd backend first in auto mode
- keep the current OpenCV/V4L2 path as a fallback only

### `src/PhysicalButtonModule.cpp`

Current compliance gaps:

- button input is working, but consumed through polling from the controller

Compliance action:

- keep the GPIO character-device implementation
- refactor how events are handed to `AppController` so button handling is more
  explicitly event-driven

### `src/MotionController.cpp`

Current compliance status:

- motion already uses a dedicated hardware boundary
- this is aligned with the current PCA9685 design

Compliance action:

- no PWM-path replacement now
- `rpi_pwm` stays reference-only unless the hardware architecture changes

## Adoption Rules

The local `compliance/` folder is reference material only. It is ignored by
git and must not become a hidden build dependency.

ARGUS now uses the first tracked approach for the relevant Bernd components:

1. vendor reviewed snapshots into `third_party/`

The local `compliance/` folder remains reference material only. ARGUS must not
depend on untracked local clones in `compliance/`.

Current direct integrations:

- `third_party/cppTimer`
- `third_party/libcamera2opencv`

Licensing and attribution for those components is documented in
`THIRD_PARTY_NOTICES.md`.

## Compliance Phases

### Phase 1: baseline documentation

- create this matrix
- create an ADR describing the adoption strategy
- use both as the gate for future compliance changes

### Phase 2: timer compliance

- integrate vendored `cppTimer`
- remove important `sleep_for(...)` usage from `AppController`
- convert demo/smoke/home pacing to timer-driven callbacks

### Phase 3: camera compliance

- add a backend boundary inside `CameraCapture`
- integrate vendored `libcamera2opencv`
- validate `--live-test` and `--full-demo` on the Pi with the new backend

### Phase 4: event-flow cleanup

- use callback-style handoff for:
  - timer ticks
  - camera frames
  - button events
- avoid introducing a large generic event bus

### Phase 5: evidence and sign-off

- document which references were adopted vs used as guidance
- record the measured runtime behavior on the Pi
- update README and PR notes once the compliance work is actually implemented

## Immediate Next Work

1. Validate the vendored `cppTimer` path on the Pi.
2. Validate the vendored `libcamera2opencv` backend on the Pi.
3. Remove the remaining non-critical polling sleep in button test if needed.
