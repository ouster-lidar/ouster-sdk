# Ensure C++14 standard is used
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(OUSTER_SDK_TEST_SOURCE "One of [STATIC|DYNAMIC]" "STATIC")
if(NOT OUSTER_SDK_TEST_SOURCE)
  message(STATUS "OUSTER_SDK_TEST_SOURCE not set, defaulting to STATIC")
  set(OUSTER_SDK_TEST_SOURCE "STATIC")
endif()

option(OUSTER_EXTERNAL_TESTS "ON/OFF" ON)
option(OUSTER_INTERNAL_TESTS "ON/OFF" ON)

message(STATUS "CPP Test Source: ${OUSTER_SDK_TEST_SOURCE}")
set(osf_includes "")
if(OUSTER_SDK_TEST_SOURCE STREQUAL "STATIC")
  set(LIB_TO_USE
    OusterSDK::ouster_client
    )
  if (BUILD_VIZ)
    list(APPEND LIB_TO_USE OusterSDK::ouster_viz)
  endif()
  if (BUILD_PCAP)
    list(APPEND LIB_TO_USE OusterSDK::ouster_pcap)
  endif()
  if (BUILD_OSF)
    list(APPEND LIB_TO_USE OusterSDK::ouster_osf)
  endif()
  if (BUILD_SENSOR)
    list(APPEND LIB_TO_USE OusterSDK::ouster_sensor)
  endif()
  if (BUILD_MAPPING)
    list(APPEND LIB_TO_USE OusterSDK::ouster_mapping)
  endif()
  # Horrible hack for the internal tests that shouldnt be a thing
  # Please remove internal tests like this
  if (BUILD_OSF)
    get_target_property(osf_includes ouster_osf INCLUDE_DIRECTORIES)
    get_target_property(osf_interface_includes ouster_osf INTERFACE_INCLUDE_DIRECTORIES)
    list(REMOVE_ITEM osf_includes ${osf_interface_includes})
  endif()
elseif(OUSTER_SDK_TEST_SOURCE STREQUAL "DYNAMIC")
  set(OUSTER_INTERNAL_TESTS OFF)
  message(WARNING "Dynamic test source detected, disabling internal tests")
  set(LIB_TO_USE OusterSDK::shared_library)
else()
  message(FATAL_ERROR "Unknown OUSTER_SDK_TEST_SOURCE: ${OUSTER_SDK_TEST_SOURCE}. Must be one of [STATIC|DYNAMIC]")
endif()

if (NOT (DEFINED OUSTER_TOP_LEVEL AND OUSTER_TOP_LEVEL))
  message(STATUS "Tests running standalone")
  get_filename_component(OUSTER_CMAKE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake" ABSOLUTE)
  set(CMAKE_PREFIX_PATH "${OUSTER_CMAKE_PATH};${CMAKE_PREFIX_PATH}")
  message("PREFIX PATH: ${CMAKE_PREFIX_PATH}")
  include(${OUSTER_CMAKE_PATH}/VcpkgEnv.cmake)
  project(ouster_sdk_tests)
  if(OUSTER_SDK_TEST_SOURCE STREQUAL "DYNAMIC")
    find_package(OusterSDK REQUIRED COMPONENTS Shared)
  else()
    find_package(OusterSDK REQUIRED)
  endif()
  include(CTest)
  find_package(GTest CONFIG REQUIRED)
  cmake_minimum_required(VERSION 3.10...3.22)
else()
  message(STATUS "Tests running as part of the SDK build")
  find_package(GTest CONFIG REQUIRED)
  if(OUSTER_SDK_TEST_SOURCE STREQUAL "DYNAMIC")
    if(NOT BUILD_SHARED_LIBRARY)
      message(FATAL_ERROR "Cannot build dynamic tests with static library build, please set BUILD_SHARED_LIBRARY=ON")
    endif()
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake")

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
message(STATUS "CPP Test Libs: ${LIB_TO_USE}")

if(NOT TARGET GTest::gtest)
  add_library(GTest::gtest ALIAS gtest)
endif()
if(NOT TARGET GTest::gtest_main)
  add_library(GTest::gtest_main ALIAS gtest_main)
endif()
find_package(Threads)

#Several deprecations since gtest 1.8
if(MSVC)
  add_compile_options(/wd4996 /wd4189)
  # Ensure windows headers do not redefine min/max/trim macros
  add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _USE_MATH_DEFINES)
else()
  add_compile_options(-Wno-deprecated-declarations -Wno-unknown-pragmas)
endif()

message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

function(ouster_internal_test target_name sources test_data)
  if(OUSTER_INTERNAL_TESTS)
    add_executable(${target_name} ${sources} test_utils.cpp)
    target_include_directories(${target_name}
    PRIVATE
      ${CMAKE_CURRENT_LIST_DIR}/../ouster_client/src
      ${CMAKE_CURRENT_LIST_DIR}/../ouster_viz/src
      ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/sophus
      ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/kiss-icp/cpp
    )
    target_link_libraries(${target_name}
      PRIVATE
        ${LIB_TO_USE}
        GTest::gtest
        GTest::gtest_main)
    CodeCoverageFunctionality(${target_name})
    add_test(
      NAME
        ${target_name}
      COMMAND
        ${target_name} --gtest_output=xml:${target_name}.xml)
    set_tests_properties(${target_name}
      PROPERTIES
      ENVIRONMENT
        DATA_DIR=${test_data}
    )
  else()
    message(WARNING "Skipping internal test '${target_name}'")
  endif()
endfunction()

function(ouster_test target_name sources test_args test_data)
  if(OUSTER_EXTERNAL_TESTS)
    add_executable(${target_name} ${sources})
    target_link_libraries(${target_name}
      PRIVATE
      ${LIB_TO_USE}
      GTest::gtest
      GTest::gtest_main)
    CodeCoverageFunctionality(${target_name})
    add_test(
      NAME
      ${target_name}
      COMMAND
      ${target_name} ${test_args} --gtest_output=xml:${target_name}.xml)
    set_tests_properties(${target_name}
      PROPERTIES
      ENVIRONMENT
      DATA_DIR=${test_data}
    )
  else()
    message(STATUS "Skipping external test '${target_name}'")
  endif()
endfunction()

ouster_internal_test(array_view_test "array_view_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_internal_test(threadsafe_queue_test "threadsafe_queue_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_internal_test(field_test "field_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_internal_test(stl_test "stl_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(zrb_test "zrb_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(zone_monitor_test "zone_monitor_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(mesh_test "mesh_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(sha256_test "sha256_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(triangle_test "triangle_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(vector_streambuf_test "vector_streambuf_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(zone_render_test "zone_render_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_internal_test(zone_header_test "zone_header_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
if (BUILD_VIZ)
ouster_internal_test(point_viz_indexed_test "point_viz_indexed_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_internal_test(zone_monitor_voxel_mesh_test "zone_monitor_voxel_mesh_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
endif()
if (BUILD_PCAP AND BUILD_MAPPING)
ouster_internal_test(slam_test "slam_test.cpp" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
endif()

ouster_test(bcompat_meta_json_test "bcompat_meta_json_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/metadata/)
ouster_test(beam_config_test "beam_config_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/zone_monitor/)
ouster_test(metadata_test "metadata_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/metadata/)
ouster_test(lidar_scan_test "lidar_scan_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/metadata/)
ouster_test(metadata_errors_test "metadata_errors_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/metadata/)
if (BUILD_PCAP)
ouster_test(pcap_test "pcap_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_test(profile_extension_test "profile_extension_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_test(fusa_profile_test "fusa_profile_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_test(scan_batcher_test "scan_batcher_test.cpp;util.h" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_test(packet_writer_test "packet_writer_test.cpp;util.h" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
if (BUILD_VIZ)
ouster_test(point_viz_test "point_viz_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
endif()
if (BUILD_SENSOR)
ouster_test(sensor_test "sensor_test.cpp" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
endif()
ouster_test(nmea_parsing_test "nmea_parsing_test.cpp;util.h" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_test(parsing_benchmark_test "parsing_benchmark_test.cpp;util.h" "" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
endif()
ouster_test(cartesian_test "cartesian_test.cpp;util.h" "${GTEST_FILTER}" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)
ouster_test(destagger_test "destagger_test.cpp" "${GTEST_FILTER}" ${CMAKE_CURRENT_LIST_DIR}/pcaps/)

ouster_test(open_source_test "open_source_test.cpp" "${GTEST_FILTER}" ${CMAKE_CURRENT_LIST_DIR}/)
if (TARGET open_source_test)
  if (BUILD_SENSOR)
    target_compile_definitions(open_source_test PRIVATE OUSTER_SENSOR)
  endif()
  if (BUILD_PCAP)
    target_compile_definitions(open_source_test PRIVATE OUSTER_PCAP)
  endif()
  if (BUILD_OSF)
    target_compile_definitions(open_source_test PRIVATE OUSTER_OSF)
  endif()
endif()

# OSF TESTS
find_package(OpenSSL REQUIRED)
function(build_osf_test_common target_name)
  target_link_libraries(${target_name}
    PRIVATE
      OpenSSL::Crypto)
  if(OUSTER_OSF_NO_THREADING)
    target_compile_definitions(${target_name} PRIVATE OUSTER_OSF_NO_THREADING)
  endif()
endfunction()

function(build_osf_test target_name sources)
  if(OUSTER_EXTERNAL_TESTS)
    ouster_test(${target_name} ${sources} "" ${CMAKE_CURRENT_LIST_DIR}/)
    build_osf_test_common(${target_name})
    #TODO Really shouldnt have tests that require this
    target_include_directories(${target_name}
      PRIVATE
      ${CMAKE_CURRENT_LIST_DIR}/../ouster_osf/src
    )
  else()
    message(STATUS "Skipping external test '${target_name}'")
  endif()
endfunction()

function(build_osf_internal_test target_name sources)
  if(OUSTER_INTERNAL_TESTS)
    ouster_internal_test(${target_name} ${sources} ${CMAKE_CURRENT_LIST_DIR}/)
    build_osf_test_common(${target_name})
    target_include_directories(${target_name}
      PRIVATE
      ${CMAKE_CURRENT_LIST_DIR}/../ouster_osf/src
      ${CMAKE_CURRENT_LIST_DIR}/../ouster_osf/include
      ${CMAKE_CURRENT_LIST_DIR}/../thirdparty
      ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/sophus
      ${CMAKE_CURRENT_LIST_DIR}/../thirdparty/jsoncons_ext
      ${osf_includes}
    )
  else()
    message(STATUS "Skipping internal test '${target_name}'")
  endif()
endfunction()

if (BUILD_OSF)
build_osf_test(chunk_test osf_tests/chunk_test.cpp)
build_osf_test(writer_test osf_tests/writer_test.cpp)
build_osf_test(writerv2_test osf_tests/writerv2_test.cpp)
build_osf_test(writer_custom_test osf_tests/writer_custom_test.cpp)
build_osf_test(file_test osf_tests/file_test.cpp)
build_osf_test(crc_test osf_tests/crc_test.cpp)
build_osf_test(file_ops_test osf_tests/file_ops_test.cpp)
build_osf_test(reader_test osf_tests/reader_test.cpp)
build_osf_test(basics_test osf_tests/basics_test.cpp)
build_osf_test(meta_streaming_info_test osf_tests/meta_streaming_info_test.cpp)
build_osf_test(buffer_offset_test osf_tests/buffer_offset_test.cpp)
build_osf_internal_test(operations_test osf_tests/operations_test.cpp)
build_osf_internal_test(png_tools_test osf_tests/png_tools_test.cpp)
endif()
