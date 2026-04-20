<p align="center">
  <img src="https://github.com/user-attachments/assets/be912344-c201-4a3f-af12-ca498a7208d0" width="550"/>
</p>

<h1 align="center">A.R.G.U.S - "Detect. Decide. Stop."</h1>

## Overview
A.R.G.U.S. (Adaptive Real-Time Guardian for Unsafe Situations) is a real-time safety supervisor for a small robotic arm.  
It watches camera input, decides whether the scene is safe, and routes motion through guardian + interlock logic so unsafe conditions trigger retract + freeze.

![ARGUS bench setup view 1](docs/images/argus-1.png)
![ARGUS bench setup view 2](docs/images/argus-2.png)
*Validated ARGUS bench setup on Raspberry Pi 5 with Pi Camera, PCA9685 and MeArm platform.*

## Bill of Materials (BOM)

### Controller
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Raspberry Pi 5 | 1 | Not tracked in repo |

### Sensors & Vision
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| Raspberry Pi Camera Module | 1 | Not tracked in repo |

### Additional Components
| Component | Quantity | Cost (£) |
|----------|---------|----------|
| MeArm test platform | 1 | Not tracked in repo |

**Total Cost:** Not tracked in the repository.

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

## What Is Validated
Current validated runtime path:

`AppController -> CameraCapture -> VisionProcessor -> GuardianStateMachine -> RobotInterlock -> MotionController`

Validated hardware:
- Raspberry Pi 5
- Raspberry Pi camera
- Adafruit PCA9685 (I2C)
- MeArm (4 servos)
- one physical operator button (GPIO24, active-low)

Validated behaviour:
- live camera safety supervision
- forbidden-layer colour unsafe trigger
- guardian freeze/recover state flow
- retract-safe before freeze hold
- manual operator acknowledge to resume
- single control contract: `space` key or physical button

## Quick Start (Fresh Pi / Debian)
### 1) Install prerequisites
```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev libopencv-contrib-dev \
  libcamera-tools libcamera-dev libturbojpeg0-dev
```

### 2) Clone and build
```bash
git clone git@github.com:ENG5220-RTEP-Team-ARGUS/ARGUS.git --recursive
cd ARGUS
cmake -S . -B build
cmake --build build -j$(nproc)
```

### 3) Run the live demo (recommended)
```bash
./scripts/live_test.sh
```

Notes:
- `live_test.sh` self-elevates with `sudo` for GPIO button access
- wrapper tries `libcamera2opencv` first, then falls back to OpenCV/V4L2
- you can force backend:
  - `ARGUS_CAMERA_BACKEND=libcamera2opencv ./scripts/live_test.sh`
  - `ARGUS_CAMERA_BACKEND=opencv ./scripts/live_test.sh`

## Live Demo Controls
Global controls:
- `space` or physical button: single control action (arm/disarm/ack based on current state)
- `0`: manual mode
- `1`: routine 1 (`SURGERY_CUT`)
- `2`: routine 2 (`BASE_SCAN`)
- `3`: routine 3 (`GRIP_PULSE`)
- `esc`: quit
- `+/-`: camera focus adjust (Pi Camera Module 3)

Manual mode (`0`) keys:
- `d/a`: base left/right
- `w/s`: forward/backward
- `i/k`: up/down
- `l/j`: gripper open/close

Expected safety flow:
1. Start in disarmed setup mode.
2. Wait until scene is safe.
3. Press `space`/button to arm and start.
4. Unsafe trigger occurs -> routine stops -> retract-safe -> freeze hold.
5. Make scene safe again.
6. Press `space`/button to acknowledge and resume.

## Common Commands
### Camera backend check
```bash
./scripts/camera_backend_check.sh
```

### Motion smoke tests
```bash
./scripts/smoke_all.sh
./scripts/smoke_base.sh
./scripts/smoke_lower.sh
./scripts/smoke_upper.sh
./scripts/smoke_grip.sh
```

### Home pose
```bash
./scripts/set_home.sh
```

### Servo tools
```bash
./scripts/servo_console.sh
./scripts/servo_drive.sh
./scripts/servo_calibrate.sh
```

### Button test
```bash
./scripts/test_button.sh
```

## Test Commands
Unit/integration tests configured in CMake:
```bash
cmake -S . -B build
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

## Latency Metrics (Runtime)
Runtime logs and dashboards expose:
- `vision_us`
- `unsafe_detect_ms`
- `freeze_pipeline_ms`
- `freeze_cmd_ms`
- `total_stop_ms`
- `ack_to_resume_ms`

Current guardian thresholds in live mode:
- freeze after `15` consecutive bad frames
- recover after `3` consecutive good frames

## Safety & Performance Metrics Justification

Benchmarked against published literature and IEC 80601-2-77.

| Metric | Measured | Target | Status | Justification |
|--------|----------|--------|--------|---------------|
| Frame Processing | `706 µs` | — | Good | Well below the 75 ms achieved by NVIDIA Holoscan [1] and 15–20 ms AI pipelines in MIS [2]. |
| Detect Unsafe | `4 ms` | ≤ 30 ms | Good | Delays >100 ms risk tissue damage [3]. Meets IEC 80601-2-77 protective stop requirements [4]. |
| Issue Freeze | `458 ms` | ≤ 900 ms | Good | Telesurgery validated up to 500 ms latency [5]; 320 ms confirmed safe over 3000 km [6]. |
| Stop Motion | `7206 ms` | ≤ 8000 ms | Good | Controlled deceleration prevents secondary tissue injury per IEC 80601-2-77 [4]. |

### References

1. NVIDIA, "Real-Time Surgical Guidance with Holoscan," 2025. [Link](https://developer.nvidia.com/blog/real-time-surgical-guidance-by-fusing-multi-modal-imaging-with-nvidia-holoscan/)
2. "AI-Based Sensorless Force Feedback in Robot-Assisted MIS," *MDPI*, 2025. [Link](https://www.mdpi.com/2078-2489/16/11/993)
3. Leung, "Engineering precision in surgical robotics," *Medical Design & Outsourcing*, 2025. [Link](https://www.medicaldesignandoutsourcing.com/engineering-precision-surgical-robotics-qnx-rtos/)
4. IEC 80601-2-77:2019; Chinzei, "Safety of Surgical Robots," *Acta Polytechnica Hungarica*, 2019. [PDF](https://acta.uni-obuda.hu/Chinzei_95.pdf)
5. Korte et al., "Impact of latency on surgical precision," *Computer Aided Surgery*, 2005. [Link](https://www.tandfonline.com/doi/full/10.3109/10929080500228654)
6. Xu et al., "Latency in robot-assisted telesurgery," *PMC*, 2024. [Link](https://pmc.ncbi.nlm.nih.gov/articles/PMC11771599/)

## Architecture and Compliance
Key docs:
- [System architecture diagram](docs/architecture/system_software_architecture.png)
- [Guardian state machine](docs/architecture/guardian_state_machine.png)
- [Threading model](docs/architecture/threading_model.png)
- [Vision safety pipeline (PDF)](docs/architecture/vision_safety_pipeline.png)
- [Compliance matrix](docs/compliance_matrix.md)
- [Compliance ADR](docs/adr/001-compliance-strategy.md)

## Wiki
Project wiki:
- https://github.com/ENG5220-RTEP-Team-ARGUS/ARGUS/wiki

## Repository Layout
```text
config/                Runtime configuration and calibration files
docs/architecture/     Architecture and wiring diagrams
docs/adr/              Architecture decision records
docs/compliance_matrix.md
include/               Headers
src/                   C++ implementation
scripts/               Pi run/test helper scripts
tests/                 CTest targets
third_party/           Vendored third-party dependencies
```
## Promotion
ARGUS public channels for demos, build updates, and outreach:
- Website: [thewebbutbeyond.github.io/argus](https://thewebbutbeyond.github.io/argus/)
- Website source: [gui/argus-app](gui/argus-app)
- Instagram: [@argus102026](https://www.instagram.com/argus102026/)
- YouTube: [@argus-w3g](https://www.youtube.com/@argus-w3g)
- LinkedIn: [A.R.G.U.S](https://www.linkedin.com/company/a-r-g-u-s)
- TikTok: [@argusxisx61](https://www.tiktok.com/@argusxisx61?_r=1&_t=ZN-95E4anYeInm)
  
## Team
| Name | Component Ownership | Key Contributions |
|------|-------------------|-------------------|
| Nathan Sidi Bakari | MotionController, AppController | Servo output path, PCA9685 integration, run modes, hardware validation |
| Patricia Munginga | VisionProcessor | Vision pipeline, colour safety logic, docs |
| Jui Ning Chin | GuardianStateMachine | FSM logic and transitions |
| Liyue Tian | CameraCapture | Camera backends and fallback behaviour |
| Nigar Baghirova | RobotInterlock | Interlock gate and motion enable/disable logic |

Project management board:
- https://github.com/orgs/ENG5220-RTEP-Team-ARGUS/projects

## Acknowledgements
- Dr Porr and Dr Chongfeng Wei (ENG5220 Real-Time Embedded Programming)
- Teaching assistants for lab and code guidance
- Bernd Porr open-source libraries:
  - [cppTimer](https://github.com/berndporr/cppTimer)
  - [libcamera2opencv](https://github.com/berndporr/libcamera2opencv)

## License
Mixed license model:
- original ARGUS code outside `third_party/`: MIT
- vendored third-party code in `third_party/`: original upstream licenses

See:
- [LICENSE](LICENSE)
- [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)
