## ARGUS v1.2 Additions

The project now has a validated Raspberry Pi hardware baseline with:
- colour-based unsafe detection
- guardian/interlock safety path
- retract-safe then freeze on unsafe
- single operator control (`space` key or physical button)

### New Manual Pages
- [Setup and Build (Pi)](01-Setup-and-Build-Pi)
- [Hardware Wiring and Power](02-Hardware-Wiring-and-Power)
- [Run Modes and Controls](03-Run-Modes-and-Controls)
- [Latency and Validation](04-Latency-and-Validation)
- [Troubleshooting](05-Troubleshooting)

### Quick Start
```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev libopencv-contrib-dev libcamera-tools libcamera-dev libturbojpeg0-dev
cmake -S . -B build
cmake --build build -j$(nproc)
./scripts/live_test.sh
```
