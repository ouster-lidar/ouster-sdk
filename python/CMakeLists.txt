cmake_minimum_required(VERSION 3.1.0)

set(OUSTER_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/.." CACHE STRING "SDK source directory")
file(TO_CMAKE_PATH "${OUSTER_SDK_PATH}" OUSTER_SDK_PATH)
message(STATUS "Ouster SDK location: ${OUSTER_SDK_PATH}")
list(APPEND CMAKE_MODULE_PATH ${OUSTER_SDK_PATH}/cmake)

project(python-ouster-sdk)
set(CMAKE_CXX_STANDARD 14)

find_package(pybind11 2.0 REQUIRED)
find_package(Eigen3 REQUIRED)
option(BUILD_VIZ "Disabled for Python build" OFF)
option(BUILD_PCAP "Enabled for Python build" ON)
find_package(OusterSDK REQUIRED)

# Several deprecations in pybind11 since 2.0. Check when changing version reqs.
if(MSVC)
  add_compile_options(/wd4996)
else()
  add_compile_options(-Wno-deprecated-declarations)
endif()

# CMAKE_LIBRARY_OUTPUT_DIRECTORY is set in setup.py to the root of the `ouster`
# namespace, but we have to provide per-target packages directories for each
# extension module here.
set(EXT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

# Note: With multi-configuration generators (like for VS), CMake automatically
# appends build-configuration suffix to *_OUTPUT_DIRECTORY properties *unless*
# they contain a generator expression, so we use a noop: $<0:>
# https://cmake.org/cmake/help/latest/prop_tgt/LIBRARY_OUTPUT_DIRECTORY.html
pybind11_add_module(_client src/cpp/client.cpp)
target_link_libraries(_client PRIVATE ouster_client)
target_include_directories(_client SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
set_target_properties(_client PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  LIBRARY_OUTPUT_DIRECTORY ${EXT_DIR}/client/$<0:>)

pybind11_add_module(_pcap src/cpp/pcap.cpp)
target_link_libraries(_pcap PRIVATE ouster_pcap)
target_include_directories(_pcap SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
set_target_properties(_pcap PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  LIBRARY_OUTPUT_DIRECTORY ${EXT_DIR}/pcap/$<0:>)
