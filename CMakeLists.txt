cmake_minimum_required(VERSION 3.10)
project(ARGUS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ARGUS_ENABLE_LIBCAMERA2OPENCV
       "Enable optional libcamera2opencv camera backend"
       OFF)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(ARGUS
    src/main.cpp
    src/AppController.cpp
    src/RealtimeTimer.cpp
    src/GuardianStateMachine.cpp
    src/VisionProcessor.cpp
    src/CameraCapture.cpp
    src/MotionController.cpp
    src/RobotInterlock.cpp
    src/PhysicalButtonModule.cpp
)

target_include_directories(ARGUS PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ARGUS
    ${OpenCV_LIBS}
    Threads::Threads
)

if(ARGUS_ENABLE_LIBCAMERA2OPENCV)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ARGUS_LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
    pkg_check_modules(ARGUS_TURBOJPEG REQUIRED IMPORTED_TARGET libturbojpeg)

    find_path(ARGUS_LIBCAM2OPENCV_INCLUDE_DIR
              NAMES libcam2opencv.h)
    find_library(ARGUS_LIBCAM2OPENCV_LIBRARY
                 NAMES cam2opencv)

    if(NOT ARGUS_LIBCAM2OPENCV_INCLUDE_DIR OR NOT ARGUS_LIBCAM2OPENCV_LIBRARY)
        message(FATAL_ERROR
                "ARGUS_ENABLE_LIBCAMERA2OPENCV=ON but cam2opencv was not found. "
                "Install libcamera2opencv first or configure with "
                "-DARGUS_ENABLE_LIBCAMERA2OPENCV=OFF.")
    endif()

    target_compile_definitions(ARGUS PRIVATE ARGUS_HAVE_LIBCAM2OPENCV=1)
    target_include_directories(ARGUS PRIVATE
        ${ARGUS_LIBCAM2OPENCV_INCLUDE_DIR}
    )
    target_link_libraries(ARGUS
        ${ARGUS_LIBCAM2OPENCV_LIBRARY}
        PkgConfig::ARGUS_LIBCAMERA
        PkgConfig::ARGUS_TURBOJPEG
    )
endif()
