if (NOT (DEFINED OUSTER_TOP_LEVEL AND OUSTER_TOP_LEVEL))
  message("Tests running standalone")
  get_filename_component(OUSTER_CMAKE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake" ABSOLUTE)
  set(CMAKE_PREFIX_PATH "${OUSTER_CMAKE_PATH};${CMAKE_PREFIX_PATH}")
  include(${OUSTER_CMAKE_PATH}/VcpkgEnv.cmake)
  project(ouster_osf_tests)
  find_package(OusterSDK REQUIRED)
  include(CTest)
  find_package(GTest CONFIG REQUIRED)
  cmake_minimum_required(VERSION 3.10...3.22)
  set(LIB_TO_USE OusterSDK::ouster_osf OusterSDK::ouster_client OusterSDK::ouster_pcap)
else()
  message("Tests running from root")
  find_package(GTest CONFIG REQUIRED)
  set(LIB_TO_USE OusterSDK::ouster_osf OusterSDK::ouster_client OusterSDK::ouster_pcap)
endif()
message("Using Libraries: ${LIB_TO_USE}")

find_package(OpenSSL REQUIRED)

# Each test file should be in a format "<test_name>_test.cpp"
set(OSF_TESTS_SOURCES png_tools_test.cpp
                      writer_test.cpp
                      writerv2_test.cpp
                      writer_custom_test.cpp
                      file_test.cpp
                      crc_test.cpp
                      file_ops_test.cpp
                      reader_test.cpp
                      operations_test.cpp
                      basics_test.cpp
                      meta_streaming_info_test.cpp
)

message(STATUS "OSF: adding testing .... ")


# Create "osf_<test_name>" tests for every test
foreach(TEST_FULL_NAME ${OSF_TESTS_SOURCES})
  get_filename_component(TEST_FILENAME ${TEST_FULL_NAME} NAME_WE)
  add_executable(osf_${TEST_FILENAME} ${TEST_FULL_NAME})
  set_target_properties(osf_${TEST_FILENAME}
    PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")
  if (OUSTER_OSF_NO_THREADING)
    target_compile_definitions(osf_${TEST_FILENAME} PRIVATE OUSTER_OSF_NO_THREADING)
  endif()

  target_include_directories(osf_${TEST_FILENAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/../src
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty>)
  target_link_libraries(osf_${TEST_FILENAME} PRIVATE 
      ${LIB_TO_USE}
      GTest::gtest
      GTest::gtest_main
      OpenSSL::Crypto)
  CodeCoverageFunctionality(osf_${TEST_FILENAME})
  add_test(NAME osf_${TEST_FILENAME}
           COMMAND osf_${TEST_FILENAME} --gtest_output=xml:osf_${TEST_FILENAME}.xml)
  set_tests_properties(
      osf_${TEST_FILENAME}
          PROPERTIES
          ENVIRONMENT
          DATA_DIR=${CMAKE_CURRENT_LIST_DIR}/../../tests/
  )
endforeach()
