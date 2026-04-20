# Hardware Wiring and Power

## Validated hardware
- Raspberry Pi 5
- Raspberry Pi camera
- Adafruit PCA9685
- MeArm with 4 servos
- one momentary tactile button (GPIO24, active-low)
- external 6V supply for servos

## Raspberry Pi to PCA9685
- Pi `3V3` -> PCA9685 `VCC`
- Pi `GND` -> PCA9685 `GND`
- Pi `GPIO2 / SDA1` (pin 3) -> PCA9685 `SDA`
- Pi `GPIO3 / SCL1` (pin 5) -> PCA9685 `SCL`

## Servo channel map
- channel `0` -> base
- channel `4` -> lower
- channel `8` -> upper
- channel `12` -> grip

## Power rules
- servos must use external 6V supply
- do not power servos from Pi rail
- share ground across Pi, PCA9685, MeArm, battery

## Button wiring
- BCM `GPIO24` (physical pin 18) on one side
- `GND` on opposite side
- use tactile button across breadboard center gap
- avoid wiring two pins on the same side of the tactile button

## Common wiring failures
- all servos jittering: check shared ground/power first
- one joint wrong: check signal wire + horn orientation + linkage
- button stuck/inverted: re-check button orientation on breadboard
