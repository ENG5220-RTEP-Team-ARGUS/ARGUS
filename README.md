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
Runs a short, conservative arm motion sequence through the existing safety path:

`AppController -> GuardianStateMachine -> RobotInterlock -> MotionController`

```bash
./build/ARGUS --motion-smoke-test
```

This mode does not use the camera pipeline. It is meant to confirm:
- PCA9685 I2C output is working
- the MeArm channel mapping is correct
- `RobotInterlock` freezes and re-enables motion correctly
- actuation still depends on guardian/interlock state

The smoke test uses zero-centered logical joint offsets and initial sweep windows:
- base: `[-90, +90]`
- lower: `[-90, +90]`
- upper: `[-90, +90]`
- grip: `[-90, +90]`

Validated hardware mapping:
- base -> channel `0` -> MeArm `BASE`
- lower -> channel `1` -> MeArm `RIGHT`
- upper -> channel `2` -> MeArm `LEFT`
- grip -> channel `3` -> MeArm `CLAW`

Sequence:
1. Move to home pose and do one freeze/recover check there
2. For `base`, move `0 -> -90 -> +90 -> 0`
3. For `lower`, move `0 -> -90 -> +90 -> 0`
4. For `upper`, move `0 -> -90 -> +90 -> 0`
5. For `grip`, move `0 -> -90 -> +90 -> 0`

Every step is held for about 3 seconds.

Every command is clamped in software before it reaches the motion controller. If the
sequence needs to be tightened further for a mechanically sensitive build, reduce the
step constants in `src/AppController.cpp` and rerun the smoke test.

If the motion controller faults at any step, the mode shuts down the outputs and exits
without continuing the sequence.

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

The module reads GPIO `value` files under `/sys/class/gpio/gpio<N>/value` and only emits
debounced press events. On startup it will try to export the GPIO line and set it to
input mode if needed; the pull-up still has to be configured by the board/pinmux setup.

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
