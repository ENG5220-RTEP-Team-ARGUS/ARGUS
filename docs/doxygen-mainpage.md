# ARGUS API Reference

ARGUS is a Raspberry Pi safety supervisor for a small robotic arm. This site
documents the C++ interfaces behind the live safety pipeline: camera capture,
vision processing, guardian state machine, motion interlock, GPIO button input,
and PCA9685 servo control.

Use **Classes** to inspect public types and methods. Use **Files** to browse
headers and source files by module.

Good starting points:

- `AppController` - top-level live/demo mode orchestration
- `CameraCapture` - Pi camera frame acquisition
- `VisionProcessor` - per-frame safety evaluation
- `GuardianStateMachine` - freeze/recovery decision logic
- `RobotInterlock` - safety gate before hardware motion
- `MotionController` - PCA9685 servo output control
- `PhysicalButtonModule` - GPIO operator button input
