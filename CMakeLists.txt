cmake_minimum_required(VERSION 3.10...3.22)
if (WIN32)
  set(CMAKE_OBJECT_PATH_MAX 260)
endif ()
if(DEFINED CMAKE_CXX_COMPILER_LAUNCHER )
  message("Using CXX Launcher: ${CMAKE_CXX_COMPILER_LAUNCHER}")
  if( DEFINED ENV{CCACHE_DIR} )
    message("Using CCACHE_DIR: $ENV{CCACHE_DIR}")
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(DefaultBuildType)

# configure vcpkg from environment variables, if present
include(VcpkgEnv)

# ==== Project Name ====
project(ouster-sdk VERSION 0.15.1)

# generate version header
set(OusterSDK_VERSION_STRING 0.15.1)
include(VersionGen)

# ==== Options ====
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)
option(BUILD_SENSOR "Build Ouster Sensor library." ON)
option(BUILD_PCAP "Build pcap utils." ON)
option(BUILD_OSF "Build Ouster OSF library." ON)
option(BUILD_VIZ "Build Ouster visualizer." ON)
option(BUILD_MAPPING "Build mapping." ON)
option(BUILD_PYTHON_MODULE "Build python module (should not use this except in special instances)." OFF)
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_EXAMPLES "Build C++ examples" OFF)
option(OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32 "Eigen max aligned bytes." OFF)
option(BUILD_SHARED_LIBRARY "Build shared Library." OFF)
option(BUILD_DEBIAN_FOR_GITHUB "Build debian for github ci" OFF)

# when building as a top-level project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(OUSTER_TOP_LEVEL ON)
  if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
  endif()

  if(NOT DEFINED CMAKE_CXX_EXTENSIONS)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()

  if(MSVC)
    add_compile_options(/W2 /bigobj /Zf)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
  else()
    add_compile_options(-Wall -Wextra)
  endif()

  include(CTest)
else()
  set(OUSTER_TOP_LEVEL OFF)
endif()

# === Subdirectories ===
add_subdirectory(ouster_client)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32)
  message(STATUS "Ouster SDK client: Using EIGEN_MAX_ALIGN_BYTES = 32")
  target_compile_definitions(ouster_client PUBLIC EIGEN_MAX_ALIGN_BYTES=32)
endif()

if(BUILD_SENSOR)
  add_subdirectory(ouster_sensor)
endif()

if(BUILD_PCAP)
  add_subdirectory(ouster_pcap)
endif()

if(BUILD_OSF)
  add_subdirectory(ouster_osf)
endif()

if(BUILD_VIZ)
  add_subdirectory(ouster_viz)
  add_subdirectory(thirdparty/glad)
endif()

if(BUILD_MAPPING)
  add_subdirectory(ouster_mapping)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(BUILD_SHARED_LIBRARY)
  add_subdirectory(ouster_library)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(BUILD_PYTHON_MODULE)
  set(OUSTER_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}" CACHE STRING "SDK source directory")
  add_subdirectory(python)
endif()

# ==== Packaging ====
set(CPACK_PACKAGE_CONTACT "oss@ouster.io")
set(CPACK_PACKAGE_VENDOR "Ouster")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Ouster sensor SDK")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.rst")
set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ouster")
set(CPACK_SYSTEM_NAME ${CMAKE_SYSTEM_PROCESSOR})

if(BUILD_DEBIAN_FOR_GITHUB)
  set(CPACK_DEBIAN_PACKAGE_NAME ouster-sdk)
  set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "libeigen3-dev, libtins-dev, libglfw3-dev, libpng-dev, libflatbuffers-dev")
endif()

if(OUSTER_LIBRARY_NAME)
  set(CPACK_PACKAGE_NAME "${OUSTER_LIBRARY_NAME}")
else()
  set(CPACK_PACKAGE_NAME "ouster-sdk")
endif()
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CMAKE_BUILD_TYPE}-${CPACK_PACKAGE_VERSION}-${CMAKE_HOST_SYSTEM_NAME}-${CMAKE_SYSTEM_VERSION}-${CMAKE_SYSTEM_PROCESSOR}")

set(CPACK_GENERATOR "TGZ;ZIP")
if(BUILD_DEBIAN_FOR_GITHUB)
  set(CPACK_GENERATOR "DEB;${CPACK_GENERATOR}")
endif()

# source packages
set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
set(CPACK_SOURCE_IGNORE_FILES /.git /dist)

include(CPack)

# ==== Install ====
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  OusterSDKConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(EXPORT ouster-sdk-targets
  FILE OusterSDKTargets.cmake
  NAMESPACE OusterSDK::
  DESTINATION lib/cmake/OusterSDK)

configure_file(cmake/OusterSDKConfig.cmake.in OusterSDKConfig.cmake @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/OusterSDKConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/OusterSDKConfigVersion.cmake"
  "cmake/Coverage.cmake"
  "cmake/VcpkgEnv.cmake"
  DESTINATION lib/cmake/OusterSDK)
install(FILES "cmake/FindEigen3.cmake"
  DESTINATION lib/cmake/OusterSDK)
install(FILES "cmake/Findlibtins.cmake"
  DESTINATION lib/cmake/OusterSDK)
install(FILES "cmake/FindCURL.cmake"
  DESTINATION lib/cmake/OusterSDK)
install(FILES "cmake/Findglfw3.cmake"
  DESTINATION lib/cmake/OusterSDK)
install(FILES "cmake/FindPcap.cmake"
  DESTINATION lib/cmake/OusterSDK)
if(BUILD_OSF)
  install(FILES "cmake/FindFlatbuffers.cmake"
    DESTINATION lib/cmake/OusterSDK)
endif()

install(FILES LICENSE LICENSE-bin
  DESTINATION share)
