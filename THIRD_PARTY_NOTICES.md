# Third-Party Notices

ARGUS vendors third-party course reference code from Bernd Porr under
`third_party/`.

## Vendored components

### `third_party/cppTimer`

- Origin: Bernd Porr `cppTimer`
- Files used:
  - `CppTimer.h`
  - `CppTimerStdFuncCallback.h`
- License: GNU GPL v3
- License text: [third_party/cppTimer/LICENSE](/home/nathan_sidib/Code/ENG5220-RTEP-ARGUS/ARGUS/third_party/cppTimer/LICENSE)

### `third_party/libcamera2opencv`

- Origin: Bernd Porr `libcamera2opencv`
- Files used:
  - `libcam2opencv.h`
  - `libcam2opencv.cpp`
  - `libcam2opencv_format_converter.h`
  - `libcam2opencv_format_converter.cpp`
- License: GPL-2.0-or-later
- License text: [third_party/libcamera2opencv/LICENSE](/home/nathan_sidib/Code/ENG5220-RTEP-ARGUS/ARGUS/third_party/libcamera2opencv/LICENSE)

## Notes

- These files are kept with their original upstream license headers.
- ARGUS-specific integration code lives outside `third_party/`.
- When vendored GPL components are built into the `ARGUS` executable,
  redistribution of the resulting combined program must comply with the
  applicable GPL terms of those components.
