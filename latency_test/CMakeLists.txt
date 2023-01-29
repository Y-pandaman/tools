cmake_minimum_required(VERSION 3.5)
project(latency_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -pthread")
add_definitions(-std=c++14)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(send src/send.cpp)
target_link_libraries(send ${Boost_LIBRARIES} -lboost_date_time)

add_executable(receive src/receive.cpp)
target_link_libraries(receive ${Boost_LIBRARIES})