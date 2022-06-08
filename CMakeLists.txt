cmake_minimum_required(VERSION 3.1.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_example VERSION 20220608)

# ==== Options ====
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)
option(BUILD_SHARED_LIBS "Build shared libraries." OFF)
option(BUILD_PCAP "Build pcap utils." OFF)
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_VIZ "Build Ouster visualizer." ON)
option(BUILD_EXAMPLES "Build C++ examples" OFF)

# when building as a top-level project and not as conan library
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  if(NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 11)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  if(NOT "${CMAKE_CXX_EXTENSIONS}")
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()

  if(MSVC)
    add_compile_options(/W3)
    add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
  else()
    add_compile_options(-Wall -Wextra -Werror -Wno-error=deprecated-declarations)
  endif()

  include(CTest)
endif()

# === Subdirectories ===
add_subdirectory(ouster_client)

if(BUILD_PCAP OR BUILD_EXAMPLES)
  add_subdirectory(ouster_pcap)
endif()

if(BUILD_VIZ)
  add_subdirectory(ouster_viz)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  message(STATUS "Ouster SDK client: Using EIGEN_MAX_ALIGN_BYTES = 32")
  target_compile_definitions(ouster_client PUBLIC EIGEN_MAX_ALIGN_BYTES=32)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()

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

# ==== Packaging ====
set(CPACK_PACKAGE_CONTACT "support@ouster.io")
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
set(CPACK_DEBIAN_PACKAGE_DEPENDS "cmake, libjsoncpp-dev, libtclap-dev, libeigen3-dev,
  libglfw3-dev, libglew-dev")

include(CPack)
