# Setup and Build (Pi)

This page is for fresh Raspberry Pi / Debian setup.

## Prerequisites
```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  libopencv-dev libopencv-contrib-dev \
  libcamera-tools libcamera-dev libturbojpeg0-dev
```

## Clone and build
```bash
git clone git@github.com:ENG5220-RTEP-Team-ARGUS/ARGUS.git --recursive
cd ARGUS
cmake -S . -B build
cmake --build build -j$(nproc)
```

## Validate build
```bash
ctest --test-dir build --output-on-failure
```

## First run
```bash
./scripts/live_test.sh
```

## Camera backend selection
Default wrapper behaviour:
- try `libcamera2opencv` first
- fallback to OpenCV/V4L2 path if needed

Force a backend:
```bash
ARGUS_CAMERA_BACKEND=libcamera2opencv ./scripts/live_test.sh
ARGUS_CAMERA_BACKEND=opencv ./scripts/live_test.sh
```
