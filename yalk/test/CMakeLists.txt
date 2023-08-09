cmake_minimum_required(VERSION 3.5)

# ##############################################################################
# CMake Project ##
# ##############################################################################

project(
  yalk_simple_demo
  VERSION 0.1.0
  LANGUAGES CXX)

# =========#
# Settings #
# =========#

# shared libraries
if(NOT DEFINED BUILD_SHARED_LIBS)
  message(STATUS "${PROJECT_NAME}: Building dynamically-linked binaries")
  option(BUILD_SHARED_LIBS "Build dynamically-linked binaries" ON)
  set(BUILD_SHARED_LIBS ON)
endif()

# build type
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: Defaulting build type to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# global compilations
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-O2)


include(GenerateExportHeader)

# ---------------#
# Library target #
# ---------------#

# ------------------#
# Executable targets #
# ------------------#

# add executables with project library
macro (add_app name)
  add_executable(${name} ${CMAKE_CURRENT_SOURCE_DIR}/${name}.cc)
  target_include_directories(${name} PUBLIC ${PROJECT_SOURCE_DIR}/)
  target_link_libraries(${name} ${PROJECT_SOURCE_DIR}/libyalk.so)
  install(
    TARGETS ${name}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endmacro()

add_app(test)

