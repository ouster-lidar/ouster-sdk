cmake_minimum_required(VERSION 3.1.0)

#Several deprecations since gtest 1.8
if(MSVC)
    add_compile_options(/wd4996 /wd4189)
else()
    add_compile_options(-Wno-deprecated-declarations -Wno-unknown-pragmas)
endif()

find_package(GTest)

add_executable(bcompat_meta_json_test bcompat_meta_json_test.cpp bcompat_sensor_info_data.h)

target_link_libraries(bcompat_meta_json_test OusterSDK::ouster_client GTest::GTest GTest::Main)

add_test(NAME bcompat_meta_json_test COMMAND bcompat_meta_json_test --gtest_output=xml:bcompat_meta_json_test.xml)
set_tests_properties(
    bcompat_meta_json_test
        PROPERTIES
        ENVIRONMENT
        DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/assets/
)
