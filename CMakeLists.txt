set(OpenCV_DIR "C:/Users/patri/opencv/opencv/build")
cmake_minimum_required(VERSION 3.10)
project(ARGUS)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(ARGUS
    src/main.cpp
    src/VisionProcessor.cpp
)

target_link_libraries(ARGUS
    ${OpenCV_LIBS}
)