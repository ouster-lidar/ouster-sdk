cmake_minimum_required(VERSION 3.1.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_example)

# ==== Options ====
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)
option(BUILD_SHARED_LIBS "Build shared libraries." OFF)
option(BUILD_VIZ "Build Ouster visualizer." ON)
option(BUILD_PCAP "Build pcap utils." OFF)

set(CMAKE_CXX_STANDARD 11)
if(MSVC)
  add_compile_options(/W3)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

# === Subdirectories ===
add_subdirectory(ouster_client)

if(BUILD_PCAP)
  add_subdirectory(ouster_pcap)
endif()

if(BUILD_VIZ)
  add_subdirectory(ouster_viz)
endif()
