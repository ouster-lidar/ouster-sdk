cmake_minimum_required(VERSION 3.1.0)

find_package(GTest REQUIRED)

# Each test file should be in a format "<test_name>_test.cpp"
set(OSF_TESTS_SOURCES png_tools_test.cpp
                      writer_test.cpp
                      writer_custom_test.cpp
                      file_test.cpp
                      crc_test.cpp
                      file_ops_test.cpp
                      reader_test.cpp
                      operations_test.cpp
                      pcap_source_test.cpp
)

message(STATUS "OSF: adding testing .... ")


# Create "osf_<test_name>" tests for every test
foreach(TEST_FULL_NAME ${OSF_TESTS_SOURCES})
  get_filename_component(TEST_FILENAME ${TEST_FULL_NAME} NAME_WE)
  add_executable(osf_${TEST_FILENAME} ${TEST_FULL_NAME})
  set_target_properties(osf_${TEST_FILENAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")
  target_include_directories(osf_${TEST_FILENAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../src)
  target_link_libraries(osf_${TEST_FILENAME} ouster_osf
                                             GTest::gtest GTest::gtest_main)
  add_test(NAME osf_${TEST_FILENAME}
           COMMAND osf_${TEST_FILENAME} --gtest_output=xml:osf_${TEST_FILENAME}.xml)
  set_tests_properties(
      osf_${TEST_FILENAME}
          PROPERTIES
          ENVIRONMENT
          DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/../../tests/
  )
endforeach()