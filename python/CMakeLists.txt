cmake_minimum_required(VERSION 3.10...3.22)

set(OUSTER_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/.." CACHE STRING "SDK source directory")
file(TO_CMAKE_PATH "${OUSTER_SDK_PATH}" OUSTER_SDK_PATH)
message(STATUS "Ouster SDK location: ${OUSTER_SDK_PATH}")
list(APPEND CMAKE_MODULE_PATH ${OUSTER_SDK_PATH}/cmake)

# configure vcpkg from environment variables, if present
include(VcpkgEnv)
include(Coverage)

project(python-ouster-sdk)

# ==== Options ====
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  add_compile_options(/W2 /wd4996)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(-Wall -Wextra -Wno-error=deprecated-declarations)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Parallelize linking optimization for GCC
    add_link_options(-flto=auto)
  endif()
endif()

option(BUILD_SENSOR "Enabled for Python build" ON)
option(BUILD_VIZ "Enabled for Python build" ON)
option(BUILD_OSF "Build OSF library." ON)
option(BUILD_PCAP "Enabled for Python build" ON)
option(BUILD_MAPPING "Enabled for Python build" ON)
option(SKIP_SDK_FIND "Skip finding the sdk" OFF)

# ==== Requirements ====
find_package(Pybind11Internal)
find_package(Eigen3 REQUIRED)
find_package(Flatbuffers NAMES Flatbuffers FlatBuffers)
if(NOT SKIP_SDK_FIND)
  find_package(OusterSDK REQUIRED)
endif()

# CMAKE_LIBRARY_OUTPUT_DIRECTORY is set in setup.py to the root of the `ouster`
# namespace, but we have to provide per-target packages directories for each
# extension module here.
set(EXT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdk)


# Warning: All of the python binding code must be within the same static library
# otherwise you will get random segfaults due to how pybind11 works.
#
# Note: With multi-configuration generators (like for VS), CMake automatically
# appends build-configuration suffix to *_OUTPUT_DIRECTORY properties *unless*
# they contain a generator expression, so we use a noop: $<0:>
# https://cmake.org/cmake/help/latest/prop_tgt/LIBRARY_OUTPUT_DIRECTORY.html
pybind11_add_module(_bindings
  src/cpp/main.cpp
  src/cpp/_viz.cpp
  src/cpp/_pcap.cpp
  src/cpp/_osf.cpp
  src/cpp/_mapping.cpp
  src/cpp/client/main.cpp
  src/cpp/client/zone_monitor.cpp
  src/cpp/client/packet.cpp
  src/cpp/client/scan_source.cpp
  src/cpp/client/lidar_scan.cpp
  src/cpp/client/field.cpp
  src/cpp/client/data.cpp
  src/cpp/client/client.cpp
  src/cpp/client/misc.cpp
  src/cpp/client/processing.cpp
  )

target_link_libraries(_bindings
  PRIVATE
    ouster_client
    ouster_sensor
    ouster_build
    ouster_pcap
    ouster_osf
    ouster_viz
    ouster_mapping
    flatbuffers::flatbuffers
  )
CodeCoverageFunctionality(_bindings)

target_include_directories(_bindings SYSTEM
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/sophus>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/kiss-icp/cpp>
    ${CMAKE_CURRENT_LIST_DIR}/src/cpp
  )

set_target_properties(_bindings PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  LIBRARY_OUTPUT_DIRECTORY ${EXT_DIR}/$<0:>)
target_compile_definitions(_bindings PRIVATE
  PYBIND11_DETAILED_ERROR_MESSAGES=TRUE
  )
