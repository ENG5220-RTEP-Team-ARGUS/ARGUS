cmake_minimum_required(VERSION 3.10)
project(ARGUS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

option(ARGUS_ENABLE_LIBCAMERA2OPENCV
       "Enable bundled libcamera2opencv camera backend when dependencies are available"
       ON)

find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR
        "OpenCV not found.\n"
        "Install prerequisites on Debian/Raspberry Pi OS:\n"
        "  sudo apt update\n"
        "  sudo apt install -y build-essential cmake pkg-config libopencv-dev libopencv-contrib-dev\n")
endif()
find_package(Threads REQUIRED)

# Main ARGUS executable
add_executable(ARGUS
    src/main.cpp
    src/AppController.cpp
    src/GuardianStateMachine.cpp
    src/VisionProcessor.cpp
    src/CameraCapture.cpp
    src/MotionController.cpp
    src/RobotInterlock.cpp
    src/PhysicalButtonModule.cpp
)

target_include_directories(ARGUS PRIVATE
    include
    third_party/cppTimer
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ARGUS
    ${OpenCV_LIBS}
    Threads::Threads
)

if(ARGUS_ENABLE_LIBCAMERA2OPENCV)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(ARGUS_LIBCAMERA QUIET IMPORTED_TARGET libcamera)
        pkg_check_modules(ARGUS_TURBOJPEG QUIET IMPORTED_TARGET libturbojpeg)
    endif()

    if(PkgConfig_FOUND AND ARGUS_LIBCAMERA_FOUND AND ARGUS_TURBOJPEG_FOUND)
        add_library(argus_cam2opencv STATIC
            third_party/libcamera2opencv/libcam2opencv.cpp
            third_party/libcamera2opencv/libcam2opencv_format_converter.cpp
        )

        target_include_directories(argus_cam2opencv PUBLIC
            third_party/libcamera2opencv
            ${ARGUS_LIBCAMERA_INCLUDE_DIRS}
            ${ARGUS_TURBOJPEG_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
        )
        target_link_libraries(argus_cam2opencv PUBLIC
            PkgConfig::ARGUS_LIBCAMERA
            PkgConfig::ARGUS_TURBOJPEG
            ${OpenCV_LIBS}
        )

        target_compile_definitions(ARGUS PRIVATE ARGUS_HAVE_LIBCAM2OPENCV=1)
        target_link_libraries(ARGUS argus_cam2opencv)
        target_include_directories(ARGUS PRIVATE third_party/libcamera2opencv)
    else()
        message(WARNING
                "Bundled libcamera2opencv backend disabled because required "
                "dependencies were not found (pkg-config, libcamera, or "
                "libturbojpeg). Falling back to the OpenCV/V4L2 path.")
    endif()
endif()

enable_testing()
add_subdirectory(tests)
