cmake_minimum_required(VERSION 3.5)
project(cylinder-stitch)

set ( PRJ_SRC_LIST )
set ( PRJ_HEADER_LIST )
set ( PRJ_COMPILE_OPTIONS)
set ( PRJ_COMPILE_FEATURES)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Ofast")
set(CMAKE_CXX_FLAGS "-msse4 -Ofast ${CMAKE_CXX_FLAGS}")

# OpenMP
find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# # OpenACC
# find_package(OpenACC)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenACC_CXX_FLAGS}")
# list ( APPEND PRJ_COMPILE_OPTIONS -ta=multicore,tesla )
# # list ( APPEND PRJ_COMPILE_OPTIONS -Minfo=accel )
# list ( APPEND PRJ_COMPILE_FEATURES cxx_std_17 )

file ( GLOB root_header_files "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h" )
file ( GLOB root_src_files "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" )

list ( APPEND PRJ_HEADER_LIST ${root_header_files} )
list ( APPEND PRJ_SRC_LIST ${root_src_files} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(cylinder_stitch src/cylinder_stitch.cpp)
target_link_libraries(cylinder_stitch ${OpenCV_LIBRARIES})

add_executable(color_reduce src/color_reduce.cpp)
target_link_libraries(color_reduce ${OpenCV_LIBRARIES})

add_executable(sqrt_algorithms src/sqrt_algorithms.cpp)
target_link_libraries(sqrt_algorithms)
