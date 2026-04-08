# ADR 001: Compliance Strategy for Bernd Porr Realtime References

- Status: Proposed
- Date: 2026-04-08

## Context

ARGUS already has a working Raspberry Pi hardware pipeline:

- camera capture
- vision-based safety assessment
- guardian freeze/recovery logic
- interlock-gated motion
- PCA9685-backed MeArm control
- physical operator button support

That baseline works on hardware and should be preserved.

At the same time, the module references provided by Bernd Porr represent the
expected realtime/event-driven style for ENG5220 RTEP. ARGUS is only partially
aligned with that style today. In particular:

- `src/AppController.cpp` still uses `std::this_thread::sleep_for(...)`
- top-level control remains polling-heavy
- `src/CameraCapture.cpp` uses OpenCV `VideoCapture` instead of a callback-first
  libcamera wrapper

We need a compliance path that improves alignment without destabilizing the
working system or redesigning the current architecture.

## Decision

ARGUS will adopt the Bernd Porr references in four different ways, depending on
their role:

1. `realtime_cpp_coding` is the compliance standard.
2. `cppTimer` is the first behavior-level integration target.
3. `libcamera2opencv` is the preferred optional external dependency for the
   future Raspberry Pi camera backend.
4. `cpp_event_callbacks` is architectural guidance for event handoff.
5. `rpi_pwm` is out of scope for the current PCA9685-based design.

The current architecture remains in place:

- `AppController`
- `CameraCapture`
- `VisionProcessor`
- `GuardianStateMachine`
- `RobotInterlock`
- `MotionController`
- `PhysicalButtonModule`

Compliance work will refactor implementations and interfaces inside this shape.
It will not replace the architecture wholesale.

## Consequences

### Positive

- keeps the existing working Pi demo path alive during compliance work
- gives the project a concrete standard instead of vague “follow the references”
- targets the highest-value gaps first: timing and camera backend
- avoids unnecessary motion-path redesign while the PCA9685 path is already
  working

### Negative

- compliance will be incremental rather than a one-shot rewrite
- there will be a transition period where both old and new camera/timer
  approaches may coexist
- some existing code paths will need temporary adapters during migration

## Dependency Policy

The local ignored `compliance/` folder is reference material only. ARGUS must
not depend on it at build or runtime.

If reference code is adopted, it must be brought into the tracked repository by
one of these methods:

1. vendored snapshot under a tracked third-party directory
2. documented git submodule
3. local re-implementation of the required behavior

This avoids hidden local dependencies and makes the compliance work reproducible
for reviewers and CI.

For the timer work, ARGUS will not vendor `cppTimer` directly because the local
reference carries GPL licensing while ARGUS is MIT-licensed. Instead, ARGUS
will re-implement the needed `timerfd` callback behavior locally in a tracked
wrapper.

For `libcamera2opencv`, ARGUS will not copy the local reference directly into
the tree. Instead, ARGUS will integrate it as an explicit optional external
dependency behind `CameraCapture`.

## Implementation Plan

### Step 1: document the baseline

- maintain `docs/compliance_matrix.md`
- use it to classify each reference repo as:
  - standard
  - direct dependency candidate
  - guidance
  - not applicable

### Step 2: integrate timer compliance first

Reason:

- lowest-risk technical change
- highest payoff for realtime alignment
- directly addresses a visible gap in `src/AppController.cpp`

Expected work:

- integrate `cppTimer` using a tracked dependency strategy
- use the local tracked `RealtimeTimer` wrapper as the chosen strategy
- replace important `sleep_for(...)` timing in:
  - motion-home mode
  - motion smoke test
  - full demo pacing

### Step 3: refactor camera capture toward callback-based libcamera

Reason:

- camera capture is the second largest compliance gap
- `libcamera2opencv` is already close to ARGUS needs at the interface level

Expected work:

- keep the `CameraCapture` abstraction
- refactor its implementation so backend choice is internal
- use that boundary to integrate the tracked optional `libcamera2opencv`
  backend
- validate live-test and full-demo on the Pi using the new backend

### Step 4: reduce polling and strengthen event handoff

Reason:

- aligns ARGUS more closely with `cpp_event_callbacks`
- improves structure without requiring a large event-bus redesign

Expected work:

- use callbacks for timer events
- use callbacks for camera frame delivery
- use callbacks or bounded event handoff for button input

## What Is Explicitly Not Changing Now

- `MotionController` remains PCA9685-based
- `RobotInterlock` remains the hardware safety gate
- `GuardianStateMachine` remains the freeze/recovery state machine
- `VisionProcessor` remains the source of safety assessment
- `AppController` remains the orchestration layer

This ADR is about compliance alignment, not architecture replacement.

## Acceptance Criteria

This strategy is considered successfully executed when:

1. no important control timing in `AppController` depends on
   `std::this_thread::sleep_for(...)`
2. Pi camera capture can run through the preferred callback-based backend
3. timer, camera, and button activity reach the controller through clearer
   event-style boundaries
4. ARGUS no longer depends on informal local reference clones for its behavior
5. the compliance matrix can explain, repo by repo, what was adopted and why
