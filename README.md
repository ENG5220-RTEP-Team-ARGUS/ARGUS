# A.R.G.U.S.
A.R.G.U.S. (Adaptive Real-time Guardian for Unsafe Situations) is a vision-based safety supervision layer for robotic manipulation workflows.

Current live-test pipeline:

`AppController -> CameraCapture -> VisionProcessor -> GuardianStateMachine -> RobotInterlock -> MotionController`

The project is currently focused on validating marker-based safety decisions in real camera tests on Raspberry Pi.

## What ARGUS does
- Captures live camera frames.
- Detects and validates an ArUco marker (expected ID check).
- Evaluates safety using:
  - marker presence
  - safe-zone check
  - speed check
  - (orientation logic remains scaffolded, not active)
- Runs guardian/interlock decision logic.
- Provides:
  - terminal logs per frame
  - OpenCV live debug window with state overlays
  - manual arm/disarm control for safe setup workflow

## Requirements
- C++17 compiler
- CMake (3.10+)
- OpenCV with `aruco` and `highgui` support
- Raspberry Pi camera stack (`libcamera`)

Typical Raspberry Pi packages:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev libcamera-tools
```

## Build
From repository root:

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

If CMake fails with `Could not find OpenCVConfig.cmake`, install/configure OpenCV dev packages so `find_package(OpenCV REQUIRED)` works.

## Run modes
### 1) Guardian FSM scenario demo
Runs built-in state-machine scenarios (no live camera pipeline):

```bash
./build/ARGUS
```

### 2) Live marker test (camera + vision + guardian + interlock)
Recommended on Raspberry Pi:

```bash
libcamerify ./build/ARGUS --live-test --camera-index 0 --expected-marker-id 23
```

Options:
- `--camera-index <n>`: camera index (default `0`)
- `--expected-marker-id <n>`: expected ArUco ID (default `23`)
- `--auto-ack`: auto-send operator acknowledge when frozen (test convenience)
- `--help`: print usage

### 3) Motion smoke test (hardware in the loop)
Runs a motion-only servo sweep through the existing `AppController -> MotionController`
path. Use one joint at a time when you are trying to isolate wiring, code, or
mechanical binding.

```bash
./build/ARGUS --motion-smoke-test --base
./build/ARGUS --motion-smoke-test --lower
./build/ARGUS --motion-smoke-test --upper
./build/ARGUS --motion-smoke-test --grip
```

Convenience wrappers are also provided:
- `scripts/smoke_all.sh`
- `scripts/smoke_base.sh`
- `scripts/smoke_lower.sh`
- `scripts/smoke_upper.sh`
- `scripts/smoke_grip.sh`

If you omit the joint flag, `--motion-smoke-test` runs all four joints in sequence.

The on-screen output is intentionally terse:
- `all -> 0`
- `base -> -90`
- `wait 3s`

Each run does the same simple pattern on the selected joint only:
1. Move all joints to home
2. Move the selected joint `0 -> -90 -> +90 -> 0`
3. Hold each step for about 3 seconds

This mode does not use the camera pipeline, guardian freeze path, or button input.
It is meant to confirm:
- PCA9685 I2C output is working
- the MeArm channel mapping is correct
- the selected servo can move cleanly on its own
- the arm returns to home after each sweep

The smoke test uses zero-centered logical joint offsets and initial sweep windows:
- base: `[-90, +90]`
- lower: `[-90, +90]`
- upper: `[-90, +90]`
- grip: `[-90, +90]`

Validated hardware mapping:
- base -> channel `0` -> MeArm `BASE`
- lower -> channel `4` -> MeArm `LEFT`
- upper -> channel `8` -> MeArm `RIGHT`
- grip -> channel `12` -> MeArm `CLAW`

Rear-view physical layout:
- lower servo is on the left side
- upper servo is on the right side

What you should see:
- `base` yaw left/right while `lower`, `upper`, and `grip` stay at home
- `lower` raises/lowers while the other three stay at home
- `upper` bends/extends while the other three stay at home
- `grip` opens/closes while the other three stay at home
- each selected joint should return to the neutral `0` position before the test ends

If a joint jumps in place, binds, or hits a hard stop, stop the test and inspect
that joint first: linkage tightness, servo orientation, signal wire, and 6V power
sharing. If all four scripts show the same symptom, look at the common path
first: PCA9685 power, I2C, code, or ground reference.

Every command is clamped in software before it reaches the motion controller. If the
sequence needs to be tightened further for a mechanically sensitive build, reduce the
step constants in `src/AppController.cpp` and rerun the smoke test.

If the motion controller faults at any step, the mode shuts down the outputs and exits
without continuing the sequence.

### 4) Physical button test
Runs the GPIO-backed physical button module by itself, without camera or motion.

```bash
./scripts/test_button.sh
```

or directly:

```bash
sudo -E ./build/ARGUS --button-test
```

What it prints:
- the exact gpiochip/line binding used for the ACK button
- the initial raw line state: `PRESSED` or `RELEASED`
- raw state changes as you press and release the button
- debounced semantic events such as `event=ACK_REQUEST`

If the raw state never changes, the problem is wiring, pull-up, or wrong GPIO line.
If the raw state changes but no `ACK_REQUEST` appears, the problem is debounce/event
handling.

### 5) Full pipeline hardware demo
Runs camera + vision + guardian + interlock + motion through the normal safety path.
This is the first end-to-end hardware demo on the Pi.

```bash
./scripts/full_demo.sh --camera-index 0 --expected-marker-id 23
```

or, if you prefer to invoke the binary directly:

```bash
sudo -E libcamerify ./build/ARGUS --full-demo --camera-index 0 --expected-marker-id 23
```

Convenience wrapper:
- `scripts/full_demo.sh`

The wrapper self-elevates with `sudo` if needed because the physical ACK button
backend needs access to the GPIO character device, and it will use `libcamerify`
when available so camera capture stays on the known-good V4L2 path.

What it does:
- waits for a safe camera view
- opens a small debug window showing vision, guardian, interlock, pose, and demo state
- stages `HOME`
- stays in a pre-arm observation state until the scene is safe
- lets the operator arm/start the demo only when the scene is safe
- after arm, runs the dance and uses the normal safe-again + ACK recovery path on later freezes
- then runs a conservative repeating dance:
  - `BASE +15`
  - `BASE -15`
  - `HOME`
  - `LOWER +10`
  - `LOWER -10`
  - `HOME`
  - `UPPER +10`
  - `UPPER -10`
  - `HOME`
  - `GRIP +10`
  - `GRIP -10`
  - `HOME`

Behavior:
- before arm, the window shows whether the scene is safe and ready to arm
- press `a`, `r`, or the physical button to continue once the scene is safe
- freeze immediately when the ArUco marker is lost
- when the marker is visible again, the app logs `safe again` and `waiting for ACK`
- press `a`, `r`, or the physical button to continue the normal recovery path
- motion resumes only after the guardian reaches its safe state again
- `--auto-ack` is intentionally disabled in this mode

Expected terminal messages are short:
- `pose=HOME`
- `safe and ready to arm`
- `ARM accepted -> running`
- `scene unsafe`
- `freeze: MARKER_LOST`
- `safe again`
- `waiting for ACK`
- `ACK accepted -> recovery`
- `resume`

Window control:
- `a`: continue the full demo
- `r`: continue the full demo
- `q`: quit the full demo

If the ACK button module is unavailable, the demo exits early because this mode
requires the physical button.

## Live-test controls (window)
When `--live-test` is running:
- `a`: arm guardian enforcement (only allowed if current reading is safe)
- `d`: disarm and return to setup/observation mode
- `r`: acknowledge a frozen state and let the guardian resume through its normal recovery path
- `q`: quit
- `Ctrl+C`: terminal stop fallback

System starts in `DISARMED` setup mode by default.

## Physical button module
The live controller can poll a GPIO-backed operator button. The wired button on the
current hardware is a single active-low acknowledge input:

- BCM GPIO24, physical pin 18
- one side of the button to GPIO24
- the other side to GND
- internal pull-up enabled at the board/pinmux level
- software debounce enabled in the module

Event contract:
- `ACK_REQUEST`: debounced press edge from the physical button. AppController still
  calls the guardian/interlock acknowledgement path and waits for the normal
  reset/recovery logic.

In `--full-demo`, the same physical `ACK_REQUEST` is treated as a single
`continue` action: before the demo is armed it means `arm/start`, and after a
freeze it means `acknowledge and resume` once the scene is safe again.

The button module remains semantic, not direct-control: it emits requests that
AppController routes through the existing guardian/interlock flow.

GPIO configuration is optional for overrides. The wired ACK input defaults to GPIO24,
and the module will disable itself if that line is unavailable at runtime.

Environment variables:
- `ARGUS_BUTTON_ACK_GPIO` (defaults to `24` for the wired button)
- `ARGUS_BUTTON_ARM_GPIO` and `ARGUS_BUTTON_DISARM_GPIO` are optional extras if you
  later add more operator buttons
- `ARGUS_BUTTON_ACTIVE_LOW` (`1` by default)
- `ARGUS_BUTTON_DEBOUNCE_MS` (`50` by default)

The module requests the GPIO line through `/dev/gpiochip*` using the Linux GPIO
character-device ABI and only emits debounced press events. It requests input mode
and prefers pull-up bias for the active-low ACK line.

In live mode, guardian thresholds are:
- freeze after `30` consecutive bad frames (~1 second at ~30 FPS, to avoid freezing on brief blur/noise spikes)
- recover after `3` consecutive good frames

## Recommended live-test workflow
1. Start in `DISARMED` mode.
2. Position printed marker in view.
3. Use `FOCUS_SCORE` and marker visibility to tune focus.
4. Watch overlay/terminal until current reading is safe (`CAN_ARM: YES`).
5. Press `a` to arm enforcement (`a` is rejected while `CAN_ARM: NO`).
6. Move marker out/invalid to verify unsafe/frozen transitions.
7. Press `r` or the physical ACK button to acknowledge a freeze once the scene is safe again.
8. Press `d` to return to setup mode as needed.
9. Press `q` to exit.

## What you should see
### In terminal (per frame)
- `armed=YES/NO`
- `can_arm=YES/NO`
- `vision=...`
- `focus_score=...`
- `focus=BLURRY/SOFT/SHARP`
- `guardian=...`
- `interlock=...`
- `motion_ctrl=...`
- `freeze_reason=...`

### In OpenCV window
- Live camera feed
- Overlay text:
  - `VISION: ...`
  - `GUARDIAN: ...`
  - `INTERLOCK: ...`
  - `CONTROL: ...`
  - `SAFE/UNSAFE` (or observed status while disarmed)
  - `FOCUS_SCORE: ... (BLURRY/SOFT/SHARP)`
  - expected marker ID

## Marker notes
- Default expected marker ID is `23`.
- Vision uses ArUco dictionary `DICT_6X6_250` by default.
- If your printed marker is different, pass `--expected-marker-id`.

## Focus notes
- Focus debug is always shown in live test (`FOCUS_SCORE` in terminal + overlay).
- `FOCUS_SCORE` is a quick sharpness heuristic (higher usually means sharper marker edges).

## Camera backend notes (Raspberry Pi)
- In `libcamerify` mode, capture enforces a V4L2-first open policy.
- Startup logs show which backend/path was used.
- If frames fail repeatedly, check:

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
```

- For Raspberry Pi live testing, prefer launching ARGUS with `libcamerify`:

```bash
libcamerify ./build/ARGUS --live-test --camera-index 0 --expected-marker-id 23
```

## Safety note
Live test mode is currently for software validation and operator workflow testing.
It routes freeze and enable through the PCA9685-backed motion controller path behind `RobotInterlock`.
