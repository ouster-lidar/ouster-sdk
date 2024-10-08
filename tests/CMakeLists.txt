option(ENABLE_PERFORMANCE_TESTS "Enable SpeedCheck in cartesian_test and parsing_benchmark_test" OFF)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(ENABLE_PERFORMANCE_TESTS ON)
endif()

#Several deprecations since gtest 1.8
if(MSVC)
    add_compile_options(/wd4996 /wd4189)
else()
    add_compile_options(-Wno-deprecated-declarations -Wno-unknown-pragmas)
endif()

find_package(GTest REQUIRED)
find_package(jsoncpp REQUIRED)
include(Coverage)

add_executable(bcompat_meta_json_test
  bcompat_meta_json_test.cpp)

target_link_libraries(bcompat_meta_json_test PRIVATE
  OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(bcompat_meta_json_test)

add_test(NAME bcompat_meta_json_test
  COMMAND bcompat_meta_json_test --gtest_output=xml:bcompat_meta_json_test.xml)
set_tests_properties(
    bcompat_meta_json_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/)


add_executable(metadata_test metadata_test.cpp)
target_link_libraries(metadata_test
    PUBLIC
        OusterSDK::ouster_client GTest::gtest GTest::gtest_main
    PRIVATE
        jsoncpp_lib)
CodeCoverageFunctionality(metadata_test)

add_test(NAME metadata_test COMMAND metadata_test --gtest_output=xml:metadata_test.xml)
set_tests_properties(
    metadata_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)


add_executable(lidar_scan_test lidar_scan_test.cpp)

target_link_libraries(lidar_scan_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(lidar_scan_test)

add_test(NAME lidar_scan_test COMMAND lidar_scan_test --gtest_output=xml:lidar_scan_test.xml)
set_tests_properties(
    lidar_scan_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)


message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

if (ENABLE_PERFORMANCE_TESTS)
    set(GTEST_FILTER "")
else ()
    set(GTEST_FILTER "--gtest_filter=CartesianParametrisedTestFixture.SpeedCheck")
endif ()

message(STATUS "GTEST_FILTER: ${GTEST_FILTER}")
add_executable(cartesian_test cartesian_test.cpp util.h)
target_link_libraries(cartesian_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(cartesian_test)
add_test(NAME cartesian_test COMMAND cartesian_test ${GTEST_FILTER} --gtest_output=xml:cartesian_test.xml)
set_tests_properties(
  cartesian_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)

add_executable(metadata_errors_test metadata_errors_test.cpp)

target_link_libraries(metadata_errors_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(metadata_errors_test)

add_test(NAME metadata_errors_test COMMAND metadata_errors_test --gtest_output=xml:metadata_errors_test.xml)
set_tests_properties(
    metadata_errors_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)

add_executable(pcap_test pcap_test.cpp)

target_link_libraries(pcap_test PRIVATE OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(pcap_test)

add_test(NAME pcap_test COMMAND pcap_test --gtest_output=xml:pcap_test.xml)
set_tests_properties(
    pcap_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)

add_executable(profile_extension_test profile_extension_test.cpp)

target_link_libraries(profile_extension_test PRIVATE OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(profile_extension_test)

add_test(NAME profile_extension_test COMMAND profile_extension_test --gtest_output=xml:profile_extension_test.xml)

add_executable(fusa_profile_test fusa_profile_test.cpp)

target_link_libraries(fusa_profile_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(fusa_profile_test)

add_test(NAME fusa_profile_test COMMAND fusa_profile_test --gtest_output=xml:fusa_profile_test.xml)
set_tests_properties(
    fusa_profile_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)
add_executable(parsing_benchmark_test parsing_benchmark_test.cpp util.h)
target_link_libraries(parsing_benchmark_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)

if(ENABLE_PERFORMANCE_TESTS)
    CodeCoverageFunctionality(parsing_benchmark_test)
    add_test(NAME parsing_benchmark_test COMMAND parsing_benchmark_test --gtest_output=xml:parsing_benchmark_test.xml)

    set_tests_properties(
        parsing_benchmark_test
            PROPERTIES
            ENVIRONMENT
            DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
    )
endif()

add_executable(scan_batcher_test scan_batcher_test.cpp util.h)

target_link_libraries(scan_batcher_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(scan_batcher_test)

add_test(NAME scan_batcher_test COMMAND scan_batcher_test --gtest_output=xml:scan_batcher_test.xml)

set_tests_properties(
    scan_batcher_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)

add_executable(packet_writer_test packet_writer_test.cpp util.h)

target_link_libraries(packet_writer_test PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)
CodeCoverageFunctionality(packet_writer_test)

add_test(NAME packet_writer_test COMMAND packet_writer_test --gtest_output=xml:packet_writer_test.xml)

set_tests_properties(
    packet_writer_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)

add_executable(array_view_test array_view_test.cpp)
target_link_libraries(array_view_test OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
add_test(NAME array_view_test COMMAND array_view_test --gtest_output=xml:array_view_test.xml)

add_executable(field_test field_test.cpp)
target_link_libraries(field_test OusterSDK::ouster_client GTest::gtest GTest::gtest_main)
add_test(NAME field_test COMMAND field_test --gtest_output=xml:field_test.xml)
if(NOT MSVC)
    target_compile_options(field_test PRIVATE -Wno-unused-variable -Wno-unused-but-set-variable)
endif()

add_executable(point_viz_test point_viz_test.cpp)
target_link_libraries(point_viz_test OusterSDK::ouster_client
    OusterSDK::ouster_viz GTest::gtest GTest::gtest_main
)
add_test(NAME point_viz_test COMMAND point_viz_test --gtest_output=xml:point_viz_test.xml)
