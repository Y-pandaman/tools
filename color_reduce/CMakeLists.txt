cmake_minimum_required(VERSION 3.5)
project(color-reduce)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(color-reduce color-reduce.cpp)
target_link_libraries(color-reduce ${OpenCV_LIBRARIES})
