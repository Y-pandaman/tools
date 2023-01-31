cmake_minimum_required(VERSION 3.5)
project(udp2can)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -pthread")
add_definitions(-std=c++14)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        )
aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

add_executable(udp2can ${DIR_SRCS})
target_link_libraries(udp2can 
                        ${Boost_LIBRARIES}
                        dl
                        )