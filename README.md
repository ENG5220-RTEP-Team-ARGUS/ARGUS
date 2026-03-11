# A.R.G.U.S.
A.R.G.U.S. (Adaptive Real-time Guardian for Unsafe Situations) is a vision-based safety supervision layer for robotic manipulation workflows.

Current live-test pipeline:

`CameraCapture -> VisionProcessor -> GuardianStateMachine -> RobotInterlock`

The project is currently focused on validating marker-based safety decisions in real camera tests on Raspberry Pi.

## What ARGUS Does Today
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

## Run Modes
### 1) Guardian FSM Scenario Demo
Runs built-in state-machine scenarios (no live camera pipeline):

```bash
./build/ARGUS
```

### 2) Live Marker Test (Camera + Vision + Guardian + Interlock)
Recommended on Raspberry Pi:

```bash
libcamerify ./build/ARGUS --live-test --camera-index 0 --expected-marker-id 23
```

Options:
- `--camera-index <n>`: camera index (default `0`)
- `--expected-marker-id <n>`: expected ArUco ID (default `23`)
- `--auto-ack`: auto-send operator acknowledge when frozen (test convenience)
- `--help`: print usage

## Live-Test Controls (Window)
When `--live-test` is running:
- `a`: arm guardian enforcement (only allowed if current reading is safe)
- `d`: disarm and return to setup/observation mode
- `q`: quit

System starts in `DISARMED` setup mode by default.

## Recommended Live-Test Workflow
1. Start in `DISARMED` mode.
2. Position printed marker in view.
3. Watch overlay/terminal until current reading is safe (`CAN_ARM: YES`).
4. Press `a` to arm enforcement.
5. Move marker out/invalid to verify unsafe/frozen transitions.
6. Press `d` to return to setup mode as needed.
7. Press `q` to exit.

## What You Should See
### In terminal (per frame)
- `armed=YES/NO`
- `can_arm=YES/NO`
- `vision=...`
- `guardian=...`
- `interlock=...`
- `freeze_reason=...`

### In OpenCV window
- Live camera feed
- Overlay text:
  - `VISION: ...`
  - `GUARDIAN: ...`
  - `INTERLOCK: ...`
  - `CONTROL: ...`
  - `SAFE/UNSAFE` (or observed status while disarmed)
  - expected marker ID

## Marker Notes
- Default expected marker ID is `23`.
- Vision uses ArUco dictionary `DICT_6X6_250` by default.
- If your printed marker is different, pass `--expected-marker-id`.

## Camera Backend Notes (Raspberry Pi)
- In `libcamerify` mode, capture enforces a V4L2-first open policy.
- Startup logs show which backend/path was used.
- If frames fail repeatedly, check:

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
```

## Safety Note
Live test mode is currently for software validation and operator workflow testing.
It uses logging-oriented interlock behavior and does not require real motion actuation to validate decision flow.
