#Several deprecations since gtest 1.8
if(MSVC)
    add_compile_options(/wd4996 /wd4189)
else()
    add_compile_options(-Wno-deprecated-declarations -Wno-unknown-pragmas)
endif()

find_package(GTest REQUIRED)
find_package(jsoncpp REQUIRED)

add_executable(bcompat_meta_json_test
  bcompat_meta_json_test.cpp)

target_link_libraries(bcompat_meta_json_test
  OusterSDK::ouster_client GTest::gtest GTest::gtest_main)

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
add_test(NAME metadata_test COMMAND metadata_test --gtest_output=xml:metadata_test.xml)
set_tests_properties(
    metadata_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)


add_executable(lidar_scan_test lidar_scan_test.cpp)

target_link_libraries(lidar_scan_test OusterSDK::ouster_client GTest::gtest GTest::gtest_main)

add_test(NAME lidar_scan_test COMMAND lidar_scan_test --gtest_output=xml:lidar_scan_test.xml)
set_tests_properties(
    lidar_scan_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)

add_executable(cartesian_test cartesian_test.cpp util.h)

target_link_libraries(cartesian_test OusterSDK::ouster_client GTest::gtest GTest::gtest_main)

add_test(NAME cartesian_test COMMAND cartesian_test --gtest_output=xml:cartesian_test.xml)
set_tests_properties(
  cartesian_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)

add_executable(metadata_errors_test metadata_errors_test.cpp)

target_link_libraries(metadata_errors_test OusterSDK::ouster_client GTest::gtest GTest::gtest_main)

add_test(NAME metadata_errors_test COMMAND metadata_errors_test --gtest_output=xml:metadata_errors_test.xml)
set_tests_properties(
    metadata_errors_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)

add_executable(pcap_test pcap_test.cpp)

target_link_libraries(pcap_test OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)

add_test(NAME pcap_test COMMAND pcap_test --gtest_output=xml:pcap_test.xml)
set_tests_properties(
    pcap_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)

add_executable(profile_extension_test profile_extension_test.cpp)

target_link_libraries(profile_extension_test OusterSDK::ouster_client GTest::gtest GTest::gtest_main)

add_test(NAME profile_extension_test COMMAND profile_extension_test --gtest_output=xml:profile_extension_test.xml)

add_executable(fusa_profile_test fusa_profile_test.cpp)

target_link_libraries(fusa_profile_test OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)

add_test(NAME fusa_profile_test COMMAND fusa_profile_test --gtest_output=xml:fusa_profile_test.xml)
set_tests_properties(
    fusa_profile_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)

add_executable(parsing_benchmark_test parsing_benchmark_test.cpp util.h)

target_link_libraries(parsing_benchmark_test OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)

add_test(NAME parsing_benchmark_test COMMAND parsing_benchmark_test --gtest_output=xml:parsing_benchmark_test.xml)

set_tests_properties(
    parsing_benchmark_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)

add_executable(scan_batcher_test scan_batcher_test.cpp util.h)

target_link_libraries(scan_batcher_test OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)

add_test(NAME scan_batcher_test COMMAND scan_batcher_test --gtest_output=xml:parsing_benchmark_test.xml)

set_tests_properties(
    scan_batcher_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)

add_executable(packet_writer_test packet_writer_test.cpp util.h)

target_link_libraries(packet_writer_test OusterSDK::ouster_client OusterSDK::ouster_pcap GTest::gtest GTest::gtest_main)

add_test(NAME packet_writer_test COMMAND packet_writer_test --gtest_output=xml:parsing_benchmark_test.xml)

set_tests_properties(
    packet_writer_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/pcaps/
)
