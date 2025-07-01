# ==== Debug params ============
# == NOTE(pb): Left intentionally
# set(CMAKE_VERBOSE_MAKEFILE OFF)
# set(CMAKE_FIND_DEBUG_MODE OFF)

option(OUSTER_OSF_NO_MMAP "Don't use mmap(), useful for WASM targets" OFF)
option(OUSTER_OSF_NO_THREADING "Don't use threads, useful for WASM targets" OFF)

# ==== Requirements ====
find_package(ZLIB REQUIRED)
find_package(PNG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads)
include(Coverage)

# TODO: Extract to a separate FindFlatbuffers cmake file
# Flatbuffers flatc resolution and different search name 'flatbuffers` with Conan
# NOTE2[pb]: 200221007: We changed Conan cmake package to look to `flatbuffers`
#   because it started failing out of blue :idk:scream: will see.
if(NOT CONAN_EXPORTED)
  find_package(Flatbuffers REQUIRED)
else()
  find_package(flatbuffers REQUIRED)
endif()


# TODO[pb]: Move to flatbuffers 2.0 and check do we still need this???
# Using this link lib search method so to get shared .so library and not
# static in Debian systems. But it correctly find static lib in vcpkg/manylinux
# builds.
# STORY: We need to make it static (but with -fPIC) for Python bindings.
#        However in Debian packages we can only use shared libs because static
#        are not compiled with PIC. Though in vcpkg it uses static lib, 
#        which we've confirmed to be the case and what we need for manylinux.
# find_library(flatbuffers_lib NAMES flatbuffers REQUIRED)
# set(flatbuffers_lib flatbuffers::flatbuffers)

# === Flatbuffer builder functions ====
include(cmake/osf_fb_utils.cmake)

set(OSF_FB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fb)

set(FB_SOURCE_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/fb_source_generated)
set(FB_BINARY_SCHEMA_DIR ${CMAKE_CURRENT_BINARY_DIR}/fb_binary_schemas)

set(FB_MODULES_TO_BUILD os_sensor streaming ml)

# ======= Typescript Flatbuffer Generators =====
set(FB_TS_GENERATED_DIR ${FB_SOURCE_GEN_DIR}/ts)
build_ts_fb_modules(
  TARGET ts_gen
  FB_DIR "${OSF_FB_DIR}"
  FB_MODULES "${FB_MODULES_TO_BUILD}"
  SOURCE_GEN_DIR "${FB_TS_GENERATED_DIR}"
  )

# ======= Python Flatbuffer Generators =====
set(FB_PYTHON_GENERATED_DIR ${FB_SOURCE_GEN_DIR}/python)
build_py_fb_modules(
  TARGET py_gen
  FB_DIR "${OSF_FB_DIR}"
  SOURCE_GEN_DIR "${FB_PYTHON_GENERATED_DIR}"
  )

# ======= C++ Flatbuffer Generators =====
set(FB_CPP_GENERATED_DIR ${FB_SOURCE_GEN_DIR}/cpp)
build_cpp_fb_modules(
  TARGET cpp_gen
  FB_DIR "${OSF_FB_DIR}"
  FB_MODULES "${FB_MODULES_TO_BUILD}"
  SOURCE_GEN_DIR "${FB_CPP_GENERATED_DIR}"
  BINARY_SCHEMA_DIR "${FB_BINARY_SCHEMA_DIR}"
  )

# === Always generate C++ stubs ==============
#   and skip Typescript and Python code from FB specs generation
#   since they not needed during a regular OSF lib builds
add_custom_target(all_fb_gen ALL DEPENDS cpp_gen) # ts_gen py_gen

add_library(ouster_osf STATIC src/compat_ops.cpp
                              src/png_tools.cpp
                              src/basics.cpp
                              src/crc32.cpp
                              src/metadata.cpp
                              src/meta_lidar_sensor.cpp
                              src/meta_extrinsics.cpp
                              src/meta_streaming_info.cpp
                              src/stream_lidar_scan.cpp
                              src/layout_streaming.cpp
                              src/file.cpp
                              src/reader.cpp
                              src/operations.cpp
                              src/osf_scan_source.cpp
                              src/fb_utils.cpp
                              src/writer.cpp
                              src/async_writer.cpp
                              src/png_lidarscan_encoder.cpp
)
set_property(TARGET ouster_osf PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_osf PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
CodeCoverageFunctionality(ouster_osf)

if(BUILD_SHARED_LIBRARY)
  target_compile_definitions(ouster_osf PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()

if (OUSTER_OSF_NO_MMAP)
  target_compile_definitions(ouster_osf PRIVATE OUSTER_OSF_NO_MMAP)
endif()

if (OUSTER_OSF_NO_THREADING)
  target_compile_definitions(ouster_osf PRIVATE OUSTER_OSF_NO_THREADING)
endif()

# Include Flatbuffers generated C++ headers
target_include_directories(ouster_osf INTERFACE ${FLATBUFFERS_INCLUDE_DIR})

target_include_directories(ouster_osf PUBLIC
    $<BUILD_INTERFACE:${FB_CPP_GENERATED_DIR}>
    $<INSTALL_INTERFACE:include/fb_generated>
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>
)
target_link_libraries(ouster_osf
  PUBLIC
    OusterSDK::ouster_client 
    OusterSDK::ouster_pcap
    flatbuffers::flatbuffers
  PRIVATE
    PNG::PNG
    ZLIB::ZLIB
)
target_include_directories(ouster_osf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
add_dependencies(ouster_osf cpp_gen)
add_library(OusterSDK::ouster_osf ALIAS ouster_osf)

# Check if ouster_client compiled with -mavx2 option and add those to ouster_osf
# If we are not matching -mavx2 compile flag Eigen lib functions might crash with
# SegFault and double free/memory corruption errors...
get_target_property(CLIENT_OPTIONS OusterSDK::ouster_client COMPILE_OPTIONS)
if(CLIENT_OPTIONS MATCHES ".*-mavx2.*")
  target_compile_options(ouster_osf PUBLIC -mavx2)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()

# ==== Install =========================================================
install(TARGETS ouster_osf
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
install(DIRECTORY ${FB_CPP_GENERATED_DIR}/ DESTINATION include/fb_generated)
