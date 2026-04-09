cmake_minimum_required(VERSION 3.10)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Add executable
add_executable(guardian_test
    src/GuardianStateMachine.cpp
    src/MetricsLogger.cpp
    src/main.cpp
)

# Link pthread
target_link_libraries(guardian_test pthread)

# Enable testing
enable_testing()

# Add test
add_test(NAME GuardianTest COMMAND guardian_test)