cmake_minimum_required(VERSION 3.10.0)

project(PackageTest CXX)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

find_package(OusterSDK REQUIRED)

add_subdirectory(../../examples examples)
