#Several deprecations since gtest 1.8
if(MSVC)
    add_compile_options(/wd4996 /wd4189)
else()
    add_compile_options(-Wno-deprecated-declarations -Wno-unknown-pragmas)
endif()

find_package(GTest REQUIRED)

add_executable(bcompat_meta_json_test
  bcompat_meta_json_test.cpp bcompat_sensor_info_data.h)

target_link_libraries(bcompat_meta_json_test
  OusterSDK::ouster_client GTest::gtest GTest::gtest_main)

add_test(NAME bcompat_meta_json_test
  COMMAND bcompat_meta_json_test --gtest_output=xml:bcompat_meta_json_test.xml)
set_tests_properties(
    bcompat_meta_json_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/)

add_executable(lidar_scan_test lidar_scan_test.cpp)

target_link_libraries(lidar_scan_test OusterSDK::ouster_client GTest::gtest GTest::gtest_main)

add_test(NAME lidar_scan_test COMMAND lidar_scan_test --gtest_output=xml:lidar_scan_test.xml)
set_tests_properties(
    lidar_scan_test
        PROPERTIES 
        ENVIRONMENT 
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/metadata/
)

add_executable(cartesian_test cartesian_test.cpp)

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
