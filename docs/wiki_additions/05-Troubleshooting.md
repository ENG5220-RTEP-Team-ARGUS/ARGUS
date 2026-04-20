# Troubleshooting

## Build fails: OpenCV not found
Install missing packages:
```bash
sudo apt update
sudo apt install -y libopencv-dev libopencv-contrib-dev
```
Then rebuild:
```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

## Live test: frame capture failed
Run backend check:
```bash
./scripts/camera_backend_check.sh
```
Try forcing backend:
```bash
ARGUS_CAMERA_BACKEND=libcamera2opencv ./scripts/live_test.sh
ARGUS_CAMERA_BACKEND=opencv ./scripts/live_test.sh
```

## Physical button not responding
- check GPIO wiring: GPIO24 to button, opposite side to GND
- verify tactile button orientation across breadboard center gap
- run:
```bash
./scripts/test_button.sh
```

## Servos jittering or weak movement
- verify external 6V supply and battery health
- confirm all grounds are shared
- check channel mapping and servo signal pins
- reset to home:
```bash
./scripts/set_home.sh
```

## Unsafe not triggering reliably
- tune live HSV + pixel-threshold sliders in status window
- ensure unsafe colour is in configured ROI
- check lighting consistency and camera focus

## Unsafe triggers too easily
- tighten HSV threshold window
- increase pixel threshold
- remove reflective background causing false positives
