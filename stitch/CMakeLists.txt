project(stitch)
cmake_minimum_required(VERSION 3.5)
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})

add_executable(stitch src/stitch.cpp)
target_link_libraries(stitch ${OpenCV_LIBRARIES})

