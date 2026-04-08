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
- [Real-Time Design & Latency](#real-time-design--latency)
- [Documentation](#documentation)
- [Social Media & PR](#social-media--pr)
- [Authors & Contributions](#authors--contributions)
- [Acknowledgements](#acknowledgements)
- [License](#license)
- [Future Work](#future-work)

---

## Overview
<p align="justify"> A.R.G.U.S (Adaptive Real-Time Guardian for Unsafe Situations) is a real-time, vision-based safety system for robotic manipulators, designed for high-risk environments such as surgical robotics and industrial automation. It continuously monitors the workspace - particularly during critical operations like instrument exchange - where unexpected motion can cause damage or injury. By analysing visual input under strict latency constraints, A.R.G.U.S. detects deviations from expected conditions and immediately triggers fail-safe interventions (e.g. hard stops) using event-driven control. This ensures deterministic, reliable interruption of motion, preventing accidents before they occur.</p>

(INsert images of the setup)

### Current validated implementation
This branch is the first fully working Raspberry Pi hardware baseline for ARGUS. The currently validated runtime path is:

`AppController -> CameraCapture -> VisionProcessor -> GuardianStateMachine -> RobotInterlock -> MotionController`

It has been exercised on real hardware with:
- Raspberry Pi 5
- Raspberry Pi camera
- PCA9685 servo driver
- MeArm test platform
- physical operator button

Current implemented capabilities:
- live camera capture on Raspberry Pi
- ArUco-based safety evaluation
- guardian freeze / recover logic
- interlock-gated motion
- PCA9685-backed servo output
- motion smoke tests
- raw servo calibration mode
- physical button test mode
- full pipeline guarded hardware demo

---

## Real-World Use Case
A.R.G.U.S is designed for **safety-critical environments**, including:

- Surgical robotics (instrument exchange safety)
- Industrial robotic arms (collision prevention)
- Human-robot collaboration systems

During operations such as **tool exchange**, unexpected motion can cause injury or system failure. A.R.G.U.S ensures the robot only operates under valid conditions, stopping instantly when anomalies occur.

### Current project focus
The current branch validates that safety workflow on a small real arm using marker-based supervision:
- safe scene required before motion
- freeze on unsafe visual state
- operator acknowledgement required for recovery
- motion always routed through the existing guardian/interlock path

---

## System Architecture
> *(Insert diagram from `/docs/architecture`)*

- Camera input → event stream
- Region of Interest (ROI) validation
- Guardian state machine
- Fail-safe controller (stop signal)

### Current validated software path
`AppController -> CameraCapture -> VisionProcessor -> GuardianStateMachine -> RobotInterlock -> MotionController`

### Current guarded hardware path
- CameraCapture acquires frames on the Pi
- VisionProcessor evaluates marker-based safety
- GuardianStateMachine decides freeze / recovery transitions
- RobotInterlock blocks or allows motion
- MotionController drives the PCA9685 servo output path
- PhysicalButtonModule emits operator button events that AppController routes through the same guarded flow

---

## Bill of Materials (BOM)

### Controller
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Raspberry Pi (Model TBD) | 1 | TBD |

### Sensors & Vision
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Camera Module | 1 | TBD |

### Additional Components
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Robotic Arm (Test Platform) | 1 | TBD |

**Total Cost:** TBD

### Current validated bench hardware
| Component | Quantity | Notes |
|----------|---------|-------|
| Raspberry Pi 5 | 1 | validated target |
| Raspberry Pi camera | 1 | used with `libcamerify` |
| Adafruit PCA9685 | 1 | I2C servo driver |
| MeArm | 1 | 4-servo arm |
| Servos | 4 | base / lower / upper / grip |
| Momentary tactile button | 1 | active-low physical continue / ACK |
| External 6V battery pack | 1 | servo power |

---

## Installation & Setup

### Requirements
- Linux (Raspberry Pi OS)
- C++17+
- CMake ≥ 3.10
- OpenCV (if used)
- libgpiod

### Install Dependencies
```bash
sudo apt update
sudo apt install cmake libgpiod-dev
```

### Current validated Raspberry Pi packages
```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev libcamera-tools
```

### Setup notes for this branch
- use `libcamerify` for Pi camera modes
- full demo self-elevates with `sudo` because the physical button uses the GPIO character-device interface
- the current default expected marker ID is `23`

---

## Hardware Wiring Cheat Sheet

### Raspberry Pi to PCA9685
- Pi `3V3` -> PCA9685 `VCC`
- Pi `GND` -> PCA9685 `GND`
- Pi `GPIO2 / SDA1` (physical pin `3`) -> PCA9685 `SDA`
- Pi `GPIO3 / SCL1` (physical pin `5`) -> PCA9685 `SCL`

### Servo power
- External `6V` battery pack -> MeArm board / servo power rail
- Pi ground, PCA9685 ground, battery ground, and MeArm ground must all be shared
- Do not power the servos from the Pi

### PCA9685 channel mapping
- `channel 0` -> `base` -> MeArm `BASE`
- `channel 4` -> `lower` -> MeArm `LEFT`
- `channel 8` -> `upper` -> MeArm `RIGHT`
- `channel 12` -> `grip` -> MeArm `CLAW`

### Rear-view arm semantics
- `base`: rotates the whole arm left / right
- `lower`: left-side servo, raises / lowers the lower link
- `upper`: right-side servo, bends / extends the upper link
- `grip`: opens / closes the claw

### Physical button
- BCM `GPIO24`, physical pin `18`
- one side of the button -> `GPIO24`
- opposite side -> `GND`
- active-low input with pull-up
- place a 4-pin tactile button across the breadboard center gap
- do not wire `GPIO24` and `GND` to two legs on the same side of the tactile button

### Camera ribbon
- use `CAM/DISP0` or `CAM/DISP1` on the Pi 5
- power the Pi off before plugging or unplugging the cable
- on the Pi side, ribbon pads face away from the latch
- on the camera side, ribbon pads face toward the camera PCB

### Quick fault isolation
- if all servos chatter together, check shared power and ground first
- if only one joint misbehaves, check that joint's linkage, orientation, and signal wire
- if the button state is stuck or inverted, check tactile button orientation first

---

## Building the Project

```bash
git clone https://github.com/YOUR_REPO.git --recursive
cd ARGUS

cmake .
make
```

### Current validated build flow
From repository root:

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

If CMake fails with `Could not find OpenCVConfig.cmake`, install or configure the OpenCV development packages so `find_package(OpenCV REQUIRED)` succeeds.

---

## Running the System

```bash
./argus_main
```

### Current validated run modes

#### 1) Guardian FSM scenario demo
Runs built-in state-machine scenarios without live camera or hardware motion:

```bash
./build/ARGUS
```

#### 2) Live marker test
Recommended on Raspberry Pi:

```bash
libcamerify ./build/ARGUS --live-test --camera-index 0 --expected-marker-id 23
```

Options:
- `--camera-index <n>`: camera index, default `0`
- `--expected-marker-id <n>`: expected ArUco ID, default `23`
- `--auto-ack`: auto-send operator acknowledge when frozen
- `--help`: print usage

#### 3) Motion smoke test
Runs a motion-only servo sweep through the existing `AppController -> MotionController` path:

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

Selected joint pattern:
1. Move all joints to home
2. Move the selected joint `0 -> -90 -> +90 -> 0`
3. Hold each step for about 3 seconds

#### 4) Move all joints to home
Moves all joints to logical `0` / home and exits:

```bash
./scripts/set_home.sh
```

or directly:

```bash
./build/ARGUS --motion-home
```

#### 5) Interactive servo console
Runs an interactive terminal console for direct joint positioning through the existing motion path:

```bash
./scripts/servo_console.sh
```

or directly:

```bash
./build/ARGUS --servo-console
```

Usage examples:
- `base 90`
- `lower -30`
- `upper 45`
- `grip 10`
- `home`
- `status`

Behavior:
- commands set one joint while keeping the others at their current logical positions
- logical range is clamped to `-90..+90`
- use `Ctrl+C` or `exit` to quit

#### 6) Servo calibration console
Runs a raw-pulse calibration console for matching physical horn angle to PCA9685 pulse ticks:

```bash
./scripts/servo_calibrate.sh
```

or directly:

```bash
./build/ARGUS --servo-calibrate
```

Usage examples:
- `base 320`
- `base +5`
- `base -5`
- `mark base 0`
- `mark base +90`
- `mark base -90`
- `summary`
- `write`

Behavior:
- commands use raw PCA9685 pulse ticks, not logical degrees
- one joint can be moved while the others stay where they are
- `mark` stores the current pulse for that joint at `-90`, `0`, or `+90`
- `summary` prints all saved calibration points
- `write` saves the current summary to `config/servo_calibration_latest.txt`
- use `Ctrl+C` or `exit` to quit

#### 7) Physical button test
Runs the GPIO-backed physical button module by itself:

```bash
./scripts/test_button.sh
```

or directly:

```bash
sudo -E ./build/ARGUS --button-test
```

#### 8) Full pipeline hardware demo
Runs camera + vision + guardian + interlock + motion through the normal safety path:

```bash
./scripts/full_demo.sh --camera-index 0 --expected-marker-id 23
```

or directly:

```bash
sudo -E libcamerify ./build/ARGUS --full-demo --camera-index 0 --expected-marker-id 23
```

Current full demo dance:
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

Live-test controls:
- `a`: arm guardian enforcement when current reading is safe
- `d`: disarm and return to setup mode
- `r`: acknowledge a frozen state and let the guardian recover
- `q`: quit

Full-demo controls:
- `a`: continue
- `r`: continue
- `q`: quit

Physical button behavior:
- in live mode, the button routes through the guardian/interlock acknowledgement path
- in full demo, the same `ACK_REQUEST` acts as `continue`
- before arm, `continue` means `arm/start`
- after a freeze, `continue` means `acknowledge and resume`

GPIO overrides:
- `ARGUS_BUTTON_ACK_GPIO` defaults to `24`
- `ARGUS_BUTTON_ARM_GPIO` and `ARGUS_BUTTON_DISARM_GPIO` are optional extras
- `ARGUS_BUTTON_ACTIVE_LOW` defaults to `1`
- `ARGUS_BUTTON_DEBOUNCE_MS` defaults to `50`

---

## Testing

```bash
make test
```

### Current validated test workflow
1. Start in `DISARMED` mode.
2. Position the printed marker in view.
3. Wait until the current reading is safe.
4. Press `a` to arm enforcement.
5. Move the marker out of view or out of the valid region to verify freeze behavior.
6. Restore a safe view.
7. Press `r` or the physical button to acknowledge and recover.
8. Press `d` to return to setup mode as needed.
9. Press `q` to exit.

### Hardware validation completed
Validated on real hardware:
- motion smoke tests
- full demo loop
- camera capture
- physical button input
- freeze / safe-again / resume path
- rebuild-from-scratch repeatability on a second day

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

---

## Project Structure

```
config/              # Configuration files
docs/architecture/   # System diagrams
include/             # Header files
src/                 # Core implementation
tests/               # Unit tests
```

### Current branch additions
```text
scripts/             # Pi helper scripts for smoke tests, calibration, button test, full demo, and home pose
build/               # out-of-tree build directory used by the validated flow
```

---

## Core Components

### Vision Processor
- Processes camera frames as events
- Extracts ROI and validates signal

### Guardian State Machine
- Encodes `SAFE` / `UNSAFE` states
- Handles transitions deterministically

### Motion Controller
- Issues stop signals
- Interfaces with robotic system

### Current implemented modules

#### AppController
- top-level orchestration for scenario demo, live test, smoke test, servo calibration, button test, and full demo

#### CameraCapture
- Pi-oriented camera acquisition with V4L2-first behavior under `libcamerify`

#### RobotInterlock
- hardware-facing gate that blocks or allows motion

#### PhysicalButtonModule
- active-low GPIO-backed operator input with software debounce and semantic events

---

## Real-Time Design & Latency

- Event-driven architecture (no polling)
- Deterministic response times
- Frame-based processing pipeline

> ⚠️ *Add measured latency results here*

### Current runtime notes
- live test freezes after `30` consecutive bad frames and recovers after `3` good frames
- full demo freezes after `1` bad frame and recovers after `3` good frames
- live test shows focus score and safety overlays to support setup and debugging

---

## Documentation

### Wiki
[https://github.com/ENG5220-RTEP-Team-ARGUS/ARGUS/wiki](https://github.com/ENG5220-RTEP-Team-ARGUS/ARGUS.wiki.git)

### Doxygen
All public classes and interfaces are documented using Doxygen.

### Camera backend notes
- in `libcamerify` mode, capture enforces a V4L2-first open policy
- startup logs show which backend/path was used
- if frames fail repeatedly, check:

```bash
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/video0
```

---

## Social Media & PR

-  [Instagram](https://www.instagram.com/argus102026/)
-  [YouTube](https://www.youtube.com/@argus-w3g)
-  [LinkedIn](https://www.linkedin.com/company/a-r-g-u-s)
-  [TikTok](https://www.tiktok.com/@argusxisx61?_r=1&_t=ZN-95E4anYeInm)

### Platform Performance Summary
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
- Funding/support sources

---

## License

*Specify license here.*

---

## Future Work

- [ ] Multi-camera integration
- [ ] Advanced risk prediction
- [ ] Full robotic system integration
- [ ] Improved latency optimisation
- [ ] More formal automated testing
- [ ] Cleaner runtime logging
- [ ] Additional operator controls
