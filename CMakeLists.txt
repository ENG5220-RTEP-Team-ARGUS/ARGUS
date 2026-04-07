cmake_minimum_required(VERSION 3.10)
project(ARGUS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(ARGUS
    src/main.cpp
    src/AppController.cpp
    src/GuardianStateMachine.cpp
    src/VisionProcessor.cpp
    src/CameraCapture.cpp
    src/MotionController.cpp
    src/RobotInterlock.cpp
)

target_include_directories(ARGUS PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(ARGUS
    ${OpenCV_LIBS}
)
