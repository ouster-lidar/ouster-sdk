cmake_minimum_required(VERSION 3.10...3.22)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(DefaultBuildType)

# configure vcpkg from environment variables, if present
include(VcpkgEnv)

# ==== Project Name ====
project(ouster_example VERSION 20230114)

# generate version header
set(OusterSDK_VERSION_STRING 0.7.1)
include(VersionGen)

# ==== Options ====
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)
option(BUILD_SHARED_LIBS "Build shared libraries." OFF)
option(BUILD_PCAP "Build pcap utils." ON)
option(BUILD_VIZ "Build Ouster visualizer." ON)
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_EXAMPLES "Build C++ examples" OFF)
option(OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32 "Eigen max aligned bytes." ON)

# when building as a top-level project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  if(NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
  endif()

  if(NOT DEFINED CMAKE_CXX_EXTENSIONS)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()

  if(MSVC)
    add_compile_options(/W2)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
  else()
    add_compile_options(-Wall -Wextra)
  endif()

  include(CTest)
endif()

# === Subdirectories ===
add_subdirectory(ouster_client)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32)
  message(STATUS "Ouster SDK client: Using EIGEN_MAX_ALIGN_BYTES = 32")
  target_compile_definitions(ouster_client PUBLIC EIGEN_MAX_ALIGN_BYTES=32)
endif()

if(BUILD_PCAP)
  add_subdirectory(ouster_pcap)
endif()

if(BUILD_VIZ)
  add_subdirectory(ouster_viz)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  add_subdirectory(tests)
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
set(CPACK_PACKAGE_NAME ouster_sdk)
set(CPACK_GENERATOR "DEB;TGZ")

# source packages
set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
set(CPACK_SOURCE_IGNORE_FILES /.git /dist)

# deb options
set(CPACK_DEBIAN_PACKAGE_NAME ouster-sdk)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
set(CPACK_DEBIAN_PACKAGE_DEPENDS
  "libjsoncpp-dev, libeigen3-dev, libtins-dev, libglfw3-dev, libglew-dev, libspdlog-dev")

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
  DESTINATION lib/cmake/OusterSDK)

install(FILES LICENSE LICENSE-bin
  DESTINATION share)
