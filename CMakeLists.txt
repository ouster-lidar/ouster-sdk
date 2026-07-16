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

option(BUILD_PERCEPTION "Enabled for Python build" OFF)
if(BUILD_PERCEPTION)
  list(APPEND VCPKG_MANIFEST_FEATURES "perception")
endif()

# configure vcpkg from environment variables, if present
include(VcpkgEnv)

# Extract version: prefer VERSION.generated from local versioning file, else repo VERSION.
set(_VERSION_GENERATED "${CMAKE_CURRENT_LIST_DIR}/VERSION.generated")
set(_VERSION_ROOT "${CMAKE_CURRENT_LIST_DIR}/VERSION")
if(EXISTS "${_VERSION_GENERATED}")
  set(VERSION_FILE_PATH "${_VERSION_GENERATED}")
  message(STATUS "OusterSDK VERSION source: generated version output (${VERSION_FILE_PATH})")
elseif(EXISTS "${_VERSION_ROOT}")
  set(VERSION_FILE_PATH "${_VERSION_ROOT}")
  message(STATUS "OusterSDK VERSION source: VERSION file (${VERSION_FILE_PATH})")
else()
  message(FATAL_ERROR "Error: No VERSION file found (expected ${_VERSION_GENERATED} or ${_VERSION_ROOT}).")
endif()

# Read the version from VERSION file
file(READ ${VERSION_FILE_PATH} VERSION_FILE_CONTENT)
string(REGEX MATCH "__cpp_version__ = \'([^\']+)\'" _ ${VERSION_FILE_CONTENT})
# ==== Set OusterSDK_VERSION_STRING ====
set(OusterSDK_VERSION_STRING "${CMAKE_MATCH_1}")
if(OusterSDK_VERSION_STRING)
  # ==== Project Name ====
  project(ouster-sdk VERSION "${OusterSDK_VERSION_STRING}")
  message(STATUS "Using OusterSDK version from VERSION file: ${OusterSDK_VERSION_STRING}")
else()
  message(FATAL_ERROR "Error: Unable to extract version from VERSION file.")
endif()

find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    message(STATUS "Selected CCache: ${CCACHE_PROGRAM}")
    get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
    set(ccacheEnv
            CCACHE_BASEDIR=${PARENT_DIR}
            CCACHE_NODISABLE=true
    )
    set(CMAKE_C_COMPILER_LAUNCHER ${CMAKE_COMMAND} -E env ${ccacheEnv} ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CMAKE_COMMAND} -E env ${ccacheEnv} ${CCACHE_PROGRAM})

    # change debug info so it is supported by ccache
    if (MSVC)
        string(REPLACE "/Zi" "/Z7" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
        string(REPLACE "/Zi" "/Z7" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
    else()
        add_compile_options(-fdebug-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=.)
    endif()
endif ()

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
option(BUILD_DEBIAN_FOR_GITHUB "Build debian for github ci." OFF)

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
    add_compile_options(/W2)
    if (NOT CCACHE_PROGRAM)
        add_compile_options(/Zf)
    endif()
    # Disable the separate mt.exe manifest step. CMake's vs_link_exe wrapper
    # invokes mt.exe after linking, but on CI the exe is already memory-mapped
    # by antivirus or applocal.ps1, causing c101008d / c10100a7 failures.
    # /MANIFEST:NO suppresses that post-link step entirely; the linker still
    # embeds a minimal default manifest on its own.
    add_link_options(/MANIFEST:NO)
  else()
    add_compile_options(-Wall -Wextra)
  endif()

  include(CTest)
else()
  set(OUSTER_TOP_LEVEL OFF)
endif()

if(MSVC)
  add_compile_options(/bigobj)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN _ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()

# === Subdirectories ===
add_subdirectory(ouster_core)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND OUSTER_USE_EIGEN_MAX_ALIGN_BYTES_32)
  message(STATUS "Ouster SDK core: Using EIGEN_MAX_ALIGN_BYTES = 32")
  target_compile_definitions(ouster_core PUBLIC EIGEN_MAX_ALIGN_BYTES=32)
endif()

add_subdirectory(ouster_algorithm)

if(BUILD_SENSOR)
  add_subdirectory(ouster_sensor)
endif()

if(BUILD_PCAP)
  message(STATUS "Building PCAP support")
  add_subdirectory(ouster_pcap)
endif()

if(BUILD_OSF)
  add_subdirectory(ouster_osf)
  add_subdirectory(thirdparty/zpng)
endif()

if(BUILD_VIZ)
    message(STATUS "Building Ouster visualizer")
  add_subdirectory(ouster_viz)
endif()

if(BUILD_MAPPING)
  add_subdirectory(ouster_mapping)
endif()

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  add_subdirectory(tests)
endif()

if(BUILD_PERCEPTION AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/_private" AND IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/_private")
  add_subdirectory(_private)
endif()

# Ouster Perception Subdirectory is just stubs, its ok to build by default
add_subdirectory(ouster_perception)

# ouster_library must be added after ouster_perception and _private so that
# the if(TARGET ouster_perception ...) checks in its CMakeLists.txt can see
# those targets and absorb their objects into shared_library.
if(BUILD_SHARED_LIBRARY)
  add_subdirectory(ouster_library)
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
install(FILES "cmake/Findlibzip.cmake"
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


if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/_private/LICENSE-freeware"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/_private/COPYRIGHT-freeware")
  install(FILES LICENSE LICENSE-bin
    DESTINATION share)
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/_private/LICENSE-freeware"
    DESTINATION share)
  set(_merged_copyright "${CMAKE_CURRENT_BINARY_DIR}/COPYRIGHT")
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/COPYRIGHT" _copyright_base)
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/_private/COPYRIGHT-freeware" _copyright_freeware)
  file(WRITE "${_merged_copyright}" "${_copyright_base}\n\n${_copyright_freeware}")
  install(FILES "${_merged_copyright}" DESTINATION share)
else()
  install(FILES LICENSE LICENSE-bin COPYRIGHT
    DESTINATION share)
endif()

if(BUILD_TESTING)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    include(${CMAKE_SOURCE_DIR}/docs/features/doc_cpp_examples.cmake)
  else()
    message(STATUS "Skipping doc_cpp_examples - currently runs on Linux")
  endif()
endif()
