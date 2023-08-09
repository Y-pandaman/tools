cmake_minimum_required(VERSION 3.5)

# ##############################################################################
# CMake Project ##
# ##############################################################################

project(
  yalk
  VERSION 0.1.0
  LANGUAGES CXX)

# =========#
# Settings #
# =========#

# options
option(BUILD_WITH_BLKID "Build with blkid" ON)
if (BUILD_WITH_BLKID)
  add_definitions(-DBUILD_WITH_BLKID)
endif()

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

# =============#
# Dependencies #
# =============#

# ----------------------#
# Third-party libraries #
# ----------------------#

# find OpenSSL
find_package(OpenSSL REQUIRED)

# find blkid
if(BUILD_WITH_BLKID)
  find_library(BLKID_LIBRARIES NAMES blkid)
  if(NOT BLKID_LIBRARIES)
    message(FATAL_ERROR "blkid not found")
  endif()
endif()

# ##############################################################################
# Build ##
# ##############################################################################

# ==============#
# Build targets #
# ==============#

include(GenerateExportHeader)

# ---------------#
# Library target #
# ---------------#

# find all source files
file(GLOB_RECURSE src_files ${PROJECT_SOURCE_DIR}/src/*.cc
     ${PROJECT_SOURCE_DIR}/src/*/*.cc)

# create library
add_library(${PROJECT_NAME} ${src_files})

# set include directories
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${OPENSSL_INCLUDE_DIR}
  PRIVATE ${PROJECT_SOURCE_DIR}/src)

# set link libraries
target_link_libraries(${PROJECT_NAME} ${OPENSSL_LIBRARIES})
if(BUILD_WITH_BLKID)
  target_link_libraries(${PROJECT_NAME} ${BLKID_LIBRARIES})
endif()

# ------------------#
# Executable targets #
# ------------------#

# add executables with project library
macro(cc_excutable dirname name)
  add_executable(${dirname}_${name}
                 ${CMAKE_CURRENT_SOURCE_DIR}/${dirname}/${name}.cc)
  target_link_libraries(${dirname}_${name} ${PROJECT_NAME})
  install(TARGETS ${dirname}_${name} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endmacro()

macro(cc_excutables dirname)
  file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/${dirname}/*.cc")
  foreach(file ${files})
    get_filename_component(name ${file} NAME_WE)
    cc_excutable(${dirname} ${name})
  endforeach()
endmacro()

cc_excutables("clis")

# -------------#
# Test targets #
# -------------#

if(NOT CMAKE_BUILD_TYPE STREQUAL "Release")
  enable_testing()

  # attempt to find Catch2
  find_package(Catch2 3 QUIET)
  # if not found, use FetchContent to download it when CMake version is high
  # enough
  if(NOT Catch2_FOUND)
    if(${CMAKE_VERSION} VERSION_LESS 3.14)
      message(
        FATAL_ERROR
          "CMake version 3.14 or higher is required to download Catch2 with FetchContent"
      )
    endif()

    include(FetchContent)
    FetchContent_Declare(
      Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG v3.3.2)
    FetchContent_MakeAvailable(Catch2)
  endif()

  # add tests with project library
  macro(zetton_cc_test dirname name)
    add_executable(${dirname}_${name}
                   ${CMAKE_CURRENT_SOURCE_DIR}/tests/${dirname}/${name}.cc)
    target_link_libraries(${dirname}_${name} ${PROJECT_NAME}
                          Catch2::Catch2WithMain)
    add_test(NAME ${dirname}_${name} COMMAND ${dirname}_${name})
    # install(TARGETS ${name} DESTINATION bin)
  endmacro()

  macro(zetton_cc_tests dirname)
    file(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/tests/${dirname}/*.cc")
    foreach(file ${files})
      get_filename_component(name ${file} NAME_WE)
      zetton_cc_test(${dirname} ${name})
    endforeach()
  endmacro()

  zetton_cc_tests("core")
  zetton_cc_tests("hwid")
  zetton_cc_tests("interface")
  zetton_cc_tests("license")
  zetton_cc_tests("protection")
  zetton_cc_tests("utils")
endif()
