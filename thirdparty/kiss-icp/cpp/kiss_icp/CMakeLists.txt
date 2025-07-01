# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 3.16...3.26)
project(kiss_icp_cpp VERSION 1.2.0 LANGUAGES CXX)

# Setup build options
option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_SOPHUS "Use system pre-installed Sophus" ON)
option(USE_SYSTEM_TSL-ROBIN-MAP "Use system pre-installed tsl_robin" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)

# ccache setup
if(USE_CCACHE)
  find_program(CCACHE_PATH ccache)
  if(CCACHE_PATH)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    message(STATUS "Using ccache: ${CCACHE_PATH}")
  endif()
endif()

# Set build type (repeat here for C++ only consumers)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

function(find_external_dependency PACKAGE_NAME TARGET_NAME)
  string(TOUPPER ${PACKAGE_NAME} PACKAGE_NAME_UP)
  set(USE_FROM_SYSTEM_OPTION "USE_SYSTEM_${PACKAGE_NAME_UP}")
  if(${${USE_FROM_SYSTEM_OPTION}})
    find_package(${PACKAGE_NAME} QUIET NO_MODULE)
  endif()
  if(NOT ${${USE_FROM_SYSTEM_OPTION}} OR NOT TARGET ${TARGET_NAME})
    set(${USE_FROM_SYSTEM_OPTION} OFF PARENT_SCOPE)
    include(${INCLUDED_CMAKE_PATH})
  endif()
endfunction()

find_package(Eigen3 REQUIRED)
find_package(TBB CONFIG REQUIRED)
if(VCPKG_TOOLCHAIN)
  find_package(tsl-robin-map CONFIG REQUIRED)
else()
  find_package(robinMap CONFIG REQUIRED PATHS ${CMAKE_CURRENT_LIST_DIR}/cmake)
endif()
find_package(Sophus REQUIRED)

include(cmake/CompilerOptions.cmake)

function(kiss_icp_include_files TARGET)
  target_include_directories(${TARGET} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/../")
endfunction()

add_subdirectory(core)
add_subdirectory(metrics)
add_subdirectory(pipeline)
