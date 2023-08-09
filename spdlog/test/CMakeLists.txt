cmake_minimum_required(VERSION 3.10)
project(spdlog_examples CXX)


find_package(spdlog REQUIRED)
find_package(Threads)


add_executable(test test.cpp)
target_link_libraries(test  spdlog::spdlog
                            ${PROJECT_SOURCE_DIR}/libspdlog.so
)