# Run Modes and Controls

## Main live mode
```bash
./scripts/live_test.sh
```

### Live controls
- `space` or physical button: single control action (arm/disarm/ack by context)
- `0`: manual mode
- `1`: `SURGERY_CUT`
- `2`: `BASE_SCAN`
- `3`: `GRIP_PULSE`
- `esc`: quit
- `+/-`: camera focus adjust (Pi Camera Module 3)

### Manual mode (`0`) keys
- `d/a`: base left/right
- `w/s`: forward/backward
- `i/k`: up/down
- `l/j`: open/close grip

## Routine behaviour
Default surgery routine:
- grip hold
- cut pass 1: forward -> down -> backward
- cut pass 2 deeper
- cut pass 3 failure-depth pass
- home

On unsafe:
1. stop routine progression
2. retract to safe pose
3. issue freeze and hold

Resume:
- requires safe scene + operator control (space/button)

## Other run modes
### Camera backend check
```bash
./scripts/camera_backend_check.sh
```

### Motion smoke test
```bash
./scripts/smoke_all.sh
```

### Home
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
