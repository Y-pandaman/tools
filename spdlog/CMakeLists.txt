cmake_minimum_required(VERSION 3.10)
project(spdlog_examples CXX)


find_package(spdlog REQUIRED)

add_library (
    spdlog  
    SHARED  
    spd_logger.cpp
)
