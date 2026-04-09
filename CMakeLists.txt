cmake_minimum_required(VERSION 3.10)
project(ARGUS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

# Main ARGUS executable
add_executable(ARGUS
    src/main.cpp
    src/VisionProcessor.cpp
)

target_link_libraries(ARGUS
    ${OpenCV_LIBS}
    pthread
)

# Testing
enable_testing()

add_executable(guardian_test
    src/GuardianStateMachine.cpp
    src/MetricsLogger.cpp
)
target_include_directories(guardian_test PRIVATE include)
target_link_libraries(guardian_test pthread)
add_test(NAME GuardianTest COMMAND guardian_test)