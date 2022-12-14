cmake_minimum_required(VERSION 2.8)
project(surf_sequential)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED
    fasthessian.cpp
    integral.cpp
    ipoint.cpp
    surf.cpp
    utils.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
)

add_executable(sequential main.cpp)
target_link_libraries(sequential ${PROJECT_NAME})