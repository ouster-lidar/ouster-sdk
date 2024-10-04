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
endif()

option(BUILD_VIZ "Enabled for Python build" ON)
option(BUILD_OSF "Build OSF library." ON)
option(BUILD_PCAP "Enabled for Python build" ON)

# ==== Requirements ====
find_package(Pybind11Internal)
find_package(Eigen3 REQUIRED)
find_package(OusterSDK REQUIRED)
find_package(spdlog REQUIRED)

# when building as a top-level project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  message(STATUS "Ouster SDK client: Using EIGEN_MAX_ALIGN_BYTES = 32")
  target_compile_definitions(ouster_client PUBLIC EIGEN_MAX_ALIGN_BYTES=32)
endif()

# CMAKE_LIBRARY_OUTPUT_DIRECTORY is set in setup.py to the root of the `ouster`
# namespace, but we have to provide per-target packages directories for each
# extension module here.
set(EXT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdk)

# Note: With multi-configuration generators (like for VS), CMake automatically
# appends build-configuration suffix to *_OUTPUT_DIRECTORY properties *unless*
# they contain a generator expression, so we use a noop: $<0:>
# https://cmake.org/cmake/help/latest/prop_tgt/LIBRARY_OUTPUT_DIRECTORY.html
pybind11_add_module(_bindings src/cpp/main.cpp src/cpp/_client.cpp src/cpp/_viz.cpp src/cpp/_pcap.cpp src/cpp/_osf.cpp)
target_link_libraries(_bindings
  PRIVATE
    ouster_client
    ouster_build
    ouster_pcap
    ouster_osf
    ouster_viz
    spdlog::spdlog)
CodeCoverageFunctionality(_bindings)
  
target_include_directories(_bindings SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
set_target_properties(_bindings PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  LIBRARY_OUTPUT_DIRECTORY ${EXT_DIR}/$<0:>)
