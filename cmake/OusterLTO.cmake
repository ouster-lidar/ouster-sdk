# Link-time optimization (LTO) for Release and MinSizeRel builds.
#
# Include after project(). Targets created afterwards in the including
# directory and its subdirectories are compiled and linked with LTO;
# OUSTER_LTO_ENABLED reports whether that happened. An explicit
# CMAKE_INTERPROCEDURAL_OPTIMIZATION setting takes precedence.
#
# The SDK only includes this when it is the top-level project. A project that
# adds the SDK with add_subdirectory() opts in by including this module before
# add_subdirectory(), so its own targets are linked with LTO as well; otherwise
# it would have to link the SDK's LTO-only static libraries without LTO.
#
# When the SDK is the top-level project (OUSTER_TOP_LEVEL), its static
# libraries are installed for downstream projects. With GCC, objects are built
# "fat": regular machine code next to the LTO bytecode. That keeps the archives
# linkable by consumers that don't use LTO or use a different GCC version.
# Clang and MSVC LTO objects hold only compiler IR, which links only with the
# same toolchain, so top-level builds with those compilers use LTO only when
# OUSTER_LTO_FOR_INSTALLED_LIBS is set, e.g. when the install is consumed by a
# build on the same machine.
include_guard(GLOBAL)

option(OUSTER_ENABLE_LTO "Use link-time optimization for Release and MinSizeRel builds" ON)
option(OUSTER_LTO_FOR_INSTALLED_LIBS
  "Use LTO in top-level Clang and MSVC builds; installed static libraries then link only with the same toolchain" OFF)

set(OUSTER_LTO_ENABLED OFF)
if(NOT OUSTER_ENABLE_LTO
   OR DEFINED CMAKE_INTERPROCEDURAL_OPTIMIZATION
   OR (CMAKE_BUILD_TYPE AND NOT CMAKE_BUILD_TYPE MATCHES "^(Release|MinSizeRel)$"))
  return()
endif()

if(OUSTER_TOP_LEVEL AND NOT CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT OUSTER_LTO_FOR_INSTALLED_LIBS)
  message(STATUS "LTO: disabled, installed static libraries would only link with this toolchain "
                 "(set OUSTER_LTO_FOR_INSTALLED_LIBS to override)")
  return()
endif()

include(CheckIPOSupported)
check_ipo_supported(RESULT _ouster_ipo_supported OUTPUT _ouster_ipo_output LANGUAGES CXX)
if(NOT _ouster_ipo_supported)
  message(STATUS "LTO: not supported by the toolchain: ${_ouster_ipo_output}")
  return()
endif()

if(OUSTER_TOP_LEVEL AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  foreach(_ouster_lang C CXX)
    list(REMOVE_ITEM CMAKE_${_ouster_lang}_COMPILE_OPTIONS_IPO -fno-fat-lto-objects)
    list(APPEND CMAKE_${_ouster_lang}_COMPILE_OPTIONS_IPO -ffat-lto-objects)
  endforeach()
endif()

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL ON)
set(OUSTER_LTO_ENABLED ON)
message(STATUS "LTO: enabled for Release and MinSizeRel builds")
