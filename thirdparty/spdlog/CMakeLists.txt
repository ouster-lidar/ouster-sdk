cmake_minimum_required(VERSION 3.5)
project(spdlog LANGUAGES CXX)

add_library(spdlog INTERFACE)
add_library(spdlog::spdlog ALIAS spdlog)

target_include_directories(spdlog INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

# Header-only, no install rules needed for embedded dependency
