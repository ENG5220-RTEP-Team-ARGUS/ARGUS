<p align="center">
  <img src="https://github.com/user-attachments/assets/be912344-c201-4a3f-af12-ca498a7208d0" width="550"/>
</p>

<h1 align="center">A.R.G.U.S - "Detect. Decide. Stop."</h1>

## Table of Contents
- [Overview](#overview)
- [Real-World Use Case](#real-world-use-case)
- [System Architecture](#system-architecture)
- [Bill of Materials (BOM)](#bill-of-materials-bom)
- [Installation & Setup](#installation--setup)
- [Hardware Wiring Cheat Sheet](#hardware-wiring-cheat-sheet)
- [Building the Project](#building-the-project)
- [Running the System](#running-the-system)
- [Testing](#testing)
- [Project Structure](#project-structure)
- [Core Components](#core-components)
- [Documentation](#documentation)
- [Social Media & PR](#social-media--pr)
- [Authors & Contributions](#authors--contributions)
- [Acknowledgements](#acknowledgements)
- [License](#license)
- [Future Work](#future-work)

---

## Overview
<p align="justify">
A.R.G.U.S. (adaptive real-time guardian for unsafe situations) is a real-time, vision-based safety supervision layer for robotic manipulators. It continuously monitors the workspace, evaluates collision or interference risk under strict latency constraints, and triggers fail-safe interventions (e.g. hard stop) via event-driven control, prioritising deterministic response and safe interruption.
</p>

This branch is the first fully working Raspberry Pi hardware baseline for ARGUS. It has been validated with a camera, a PCA9685 servo driver, a MeArm, and a physical operator button.

Current validated runtime path:

`AppController -> CameraCapture -> VisionProcessor -> GuardianStateMachine -> RobotInterlock -> MotionController`

What ARGUS currently does:
- captures live camera frames
- detects and validates an ArUco marker
- evaluates safety from marker visibility, ROI, and motion quality
- runs guardian freeze/recovery logic
- gates motion through the robot interlock
- drives the MeArm through the PCA9685 path
- supports a physical operator button for continue / acknowledge actions

---

## Real-World Use Case
A.R.G.U.S is designed for safety-critical robotic workflows, including:

- surgical robotics
- industrial robotic arms
- human-robot collaboration setups

The current project focus is a marker-supervised manipulator workflow where unsafe visual conditions must stop motion immediately and only allow recovery through the normal guarded path.

---

## System Architecture
- Camera input
- ArUco-based safety assessment
- Guardian state machine
- Robot interlock safety gate
- PCA9685-backed motion controller
- Physical operator button routed through AppController

Current software path:

`AppController -> CameraCapture -> VisionProcessor -> GuardianStateMachine -> RobotInterlock -> MotionController`

---

## Bill of Materials (BOM)

### Controller
| Component | Quantity | Notes |
|----------|---------|-------|
| Raspberry Pi 5 | 1 | Validated hardware target |

### Sensors & Vision
| Component | Quantity | Notes |
|----------|---------|-------|
| Raspberry Pi camera module | 1 | Used through `libcamera` / `libcamerify` |
| Printed ArUco marker | 1 | Default expected ID is `23` |

### Motion & I/O
| Component | Quantity | Notes |
|----------|---------|-------|
| Adafruit PCA9685 servo driver | 1 | I2C servo output path |
| MeArm test platform | 1 | 4-servo arm |
| Servos | 4 | base / lower / upper / grip |
| Momentary tactile push button | 1 | Physical continue / ACK input |
| External 6V battery pack | 1 | Servo power |

---

## Installation & Setup

### Requirements
- Linux / Raspberry Pi OS
- C++17 compiler
- CMake 3.10+
- OpenCV with `aruco` and `highgui`
- Raspberry Pi camera stack (`libcamera`)

Typical Raspberry Pi packages:

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev libcamera-tools
```

Launch notes:
- prefer running camera modes through `libcamerify`
- full demo self-elevates because the physical button uses the GPIO character device

---

## Hardware Wiring Cheat Sheet

### Raspberry Pi to PCA9685
- Pi `3V3` -> PCA9685 `VCC`
- Pi `GND` -> PCA9685 `GND`
- Pi `GPIO2 / SDA1` (physical pin `3`) -> PCA9685 `SDA`
- Pi `GPIO3 / SCL1` (physical pin `5`) -> PCA9685 `SCL`

### Servo power
- External `6V` battery pack -> MeArm board / servo power rail
- PCA9685 ground, Pi ground, battery ground, and MeArm ground must all be shared
- Do not power the servos from the Pi

### PCA9685 to MeArm mapping
- `channel 0` -> `base` -> MeArm `BASE`
- `channel 4` -> `lower` -> MeArm `LEFT`
- `channel 8` -> `upper` -> MeArm `RIGHT`
- `channel 12` -> `grip` -> MeArm `CLAW`

### Rear-view arm meaning
- `base`: rotates the whole arm left/right
- `lower`: left-side servo, raises/lowers the lower link
- `upper`: right-side servo, bends/extends the upper link
- `grip`: opens/closes the claw

### Physical ACK button
- BCM `GPIO24`, physical pin `18`
- One side of the button -> `GPIO24`
- Opposite side of the button -> `GND`
- Active-low input with pull-up enabled
- Place a 4-pin tactile button across the breadboard center gap
- Do not wire `GPIO24` and `GND` to two legs on the same side of the tactile button

### Camera ribbon
- Use `CAM/DISP0` or `CAM/DISP1` on the Pi 5
- Power the Pi off before plugging or unplugging the camera cable
- On the Pi side, ribbon pads face away from the connector latch
- On the camera side, ribbon pads face toward the camera PCB

### Quick fault isolation
- If all servos chatter together, check shared power and ground first
- If only one joint misbehaves, check that servo's linkage, orientation, and signal wire
- If the button state looks inverted or stuck, check tactile button orientation first

---

## Building the Project

From repository root:

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

If CMake fails with `Could not find OpenCVConfig.cmake`, install or configure the OpenCV development packages so `find_package(OpenCV REQUIRED)` succeeds.

---

## Running the System

### 1) Guardian FSM scenario demo
Runs built-in state-machine scenarios without live camera or hardware motion:

```bash
./build/ARGUS
```

### 2) Live marker test
Recommended on Raspberry Pi:

```bash
libcamerify ./build/ARGUS --live-test --camera-index 0 --expected-marker-id 23
```

Options:
- `--camera-index <n>`: camera index, default `0`
- `--expected-marker-id <n>`: expected ArUco ID, default `23`
- `--auto-ack`: auto-send operator acknowledge when frozen
- `--help`: print usage

### 3) Motion smoke test
Runs a motion-only servo sweep through the existing `AppController -> MotionController` path. Use one joint at a time when isolating wiring, code, or mechanical binding.

```bash
./build/ARGUS --motion-smoke-test --base
./build/ARGUS --motion-smoke-test --lower
./build/ARGUS --motion-smoke-test --upper
./build/ARGUS --motion-smoke-test --grip
```

Convenience wrappers:
- `scripts/smoke_all.sh`
- `scripts/smoke_base.sh`
- `scripts/smoke_lower.sh`
- `scripts/smoke_upper.sh`
- `scripts/smoke_grip.sh`

If you omit the joint flag, `--motion-smoke-test` runs all four joints in sequence.

Selected joint pattern:
1. Move all joints to home
2. Move the selected joint `0 -> -90 -> +90 -> 0`
3. Hold each step for about 3 seconds

This mode is meant to confirm:
- PCA9685 I2C output works
- MeArm channel mapping is correct
- each servo can move cleanly on its own
- the arm returns to home after each sweep

### 4) Physical button test
Runs the GPIO-backed physical button module by itself:

```bash
./scripts/test_button.sh
```

or directly:

```bash
sudo -E ./build/ARGUS --button-test
```

### 5) Full pipeline hardware demo
Runs camera + vision + guardian + interlock + motion through the normal safety path:

```bash
./scripts/full_demo.sh --camera-index 0 --expected-marker-id 23
```

or directly:

```bash
sudo -E libcamerify ./build/ARGUS --full-demo --camera-index 0 --expected-marker-id 23
```

Convenience wrapper:
- `scripts/full_demo.sh`

Current full demo sequence:
- `BASE +60`
- `BASE -60`
- `HOME`
- `LOWER +60`
- `LOWER -60`
- `HOME`
- `UPPER +60`
- `UPPER -60`
- `HOME`
- `GRIP +60`
- `GRIP -60`
- `HOME`

Full demo behavior:
- starts in a safe pre-arm observation state
- waits for a safe scene before arm/start
- uses `a`, `r`, or the physical button as continue input
- freezes immediately on marker loss
- requires safe-again plus operator continue to recover
- resumes only through the guardian/interlock path

Window control:
- `a`: continue
- `r`: continue
- `q`: quit

### Live-test controls
When `--live-test` is running:
- `a`: arm guardian enforcement when the current reading is safe
- `d`: disarm and return to setup mode
- `r`: acknowledge a frozen state and let the guardian recover normally
- `q`: quit
- `Ctrl+C`: terminal stop fallback

System starts in `DISARMED` mode by default.

### Physical button module
The wired operator button is a single active-low acknowledge input:
- BCM `GPIO24`, physical pin `18`
- one side of the button to GPIO24
- the other side to GND
- software debounce enabled in the module

Event contract:
- `ACK_REQUEST`: debounced press edge from the physical button

Behavior:
- in live mode, the button routes through the guardian/interlock acknowledgement path
- in full demo, the same `ACK_REQUEST` is treated as `continue`
- before full-demo arm, `continue` means `arm/start`
- after a freeze, `continue` means `acknowledge and resume` once the scene is safe again

GPIO overrides:
- `ARGUS_BUTTON_ACK_GPIO` defaults to `24`
- `ARGUS_BUTTON_ARM_GPIO` and `ARGUS_BUTTON_DISARM_GPIO` are optional extras
- `ARGUS_BUTTON_ACTIVE_LOW` defaults to `1`
- `ARGUS_BUTTON_DEBOUNCE_MS` defaults to `50`

Implementation note:
- the module uses `/dev/gpiochip*` through the Linux GPIO character-device ABI
- if the GPIO line is unavailable, the full demo exits early because the physical button is required

---

## Testing

### Recommended live-test workflow
1. Start in `DISARMED` mode.
2. Position the printed marker in view.
3. Wait until the current reading is safe.
4. Press `a` to arm enforcement.
5. Move the marker out of view or out of the valid region to verify freeze behavior.
6. Restore a safe view.
7. Press `r` or the physical button to acknowledge and recover.
8. Press `d` to return to setup mode as needed.
9. Press `q` to exit.

### Physical validation completed
The current branch has been validated on real hardware for:
- motion smoke tests
- full demo loop
- camera capture
- physical button input
- freeze / safe-again / resume behavior

### What you should see
In terminal:
- `armed=YES/NO`
- `can_arm=YES/NO`
- `vision=...`
- `guardian=...`
- `interlock=...`
- `motion_ctrl=...`
- `freeze_reason=...`

In the OpenCV window:
- live camera feed
- `VISION`
- `GUARDIAN`
- `INTERLOCK`
- `CONTROL`
- `POSE`
- `DEMO`
- `FREEZE`

### Guardian thresholds
- live test freezes after `30` consecutive bad frames and recovers after `3` good frames
- full demo freezes after `1` bad frame and recovers after `3` good frames

### Marker and focus notes
- default expected marker ID is `23`
- vision uses ArUco dictionary `DICT_6X6_250`
- if your printed marker differs, pass `--expected-marker-id`
- focus debug is shown in live test
- `FOCUS_SCORE` is a simple sharpness heuristic; higher usually means sharper marker edges

### Safety note
Live and demo modes are intended for guarded hardware validation. Freeze and enable still route through the PCA9685-backed motion path behind `RobotInterlock`; the operator button does not bypass that path.

---

## Project Structure

```text
config/              # Configuration files
docs/architecture/   # System diagrams
include/             # Public headers
scripts/             # Pi helper scripts
src/                 # Core implementation
tests/               # Test assets / future automated tests
```

---

## Core Components

### AppController
Top-level orchestration for live test, smoke test, button test, and full demo modes.

### CameraCapture
Pi-oriented camera acquisition with V4L2-first behavior under `libcamerify`.

### VisionProcessor
ArUco-based safety evaluation using marker presence, ROI, and motion checks.

### GuardianStateMachine
Encodes the freeze / reset / recover logic.

### RobotInterlock
Hardware-facing motion gate that blocks or allows actuation based on guardian state.

### MotionController
PCA9685-backed servo output path for the MeArm.

### PhysicalButtonModule
GPIO-backed active-low button input with software debounce and semantic events.

---

## Documentation

### Wiki
[https://github.com/ENG5220-RTEP-Team-ARGUS/ARGUS/wiki](https://github.com/ENG5220-RTEP-Team-ARGUS/ARGUS.wiki.git)

### Doxygen
Public classes and interfaces are intended to be documented with Doxygen as the codebase stabilises.

### Camera backend notes
- In `libcamerify` mode, capture enforces a V4L2-first open policy
- startup logs show which backend/path was used
- if frames fail repeatedly, check:

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
```

---

## Social Media & PR

- [Instagram](https://www.instagram.com/argus102026/)
- [YouTube](https://www.youtube.com/@argus-w3g)
- [LinkedIn](https://www.linkedin.com/company/a-r-g-u-s)
- [TikTok](https://www.tiktok.com/@argusxisx61?_r=1&_t=ZN-95E4anYeInm)

---

## Authors & Contributions

| Name | Contribution |
|------|-------------|
| Member 1 |  |
| Member 2 |  |
| Member 3 |  |
| Member 4 |  |
| Member 5 |  |

---

## Acknowledgements

- Course lecturers
- Lab technicians
- Funding and support contributors

---

## License

Specify license here.

---

## Future Work

- [ ] Multi-camera integration
- [ ] More formal automated testing
- [ ] Cleaner runtime logging
- [ ] Advanced risk prediction
- [ ] Additional operator controls
- [ ] Further hardware tuning and latency measurement
