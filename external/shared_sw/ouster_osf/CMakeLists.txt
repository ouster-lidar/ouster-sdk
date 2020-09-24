cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_osf)

# ==== Requirements ====
find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)
find_package(PNG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ouster_client REQUIRED)
find_package(Flatbuffers REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-mavx2 -Wall -Wextra -Werror)

# ==== Generated Files ====
set(SCHEMAS ${CMAKE_CURRENT_SOURCE_DIR}/osfChunk.fbs
            ${CMAKE_CURRENT_SOURCE_DIR}/osfHeader.fbs
            ${CMAKE_CURRENT_SOURCE_DIR}/osfSession.fbs
            )
set(SCHEMAS_COMMON ${CMAKE_CURRENT_SOURCE_DIR}/osfCommon.fbs)

set(FBS_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_private)
set(FBS_GENERATED_COMMON_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_common)
file(MAKE_DIRECTORY ${FBS_GENERATED_DIR})
file(MAKE_DIRECTORY ${FBS_GENERATED_COMMON_DIR})


# TODO[pb]: Use --scoped-enums for generated C++?

# ==== Generate private schemas fbs =====
foreach(FBS ${SCHEMAS})
  get_filename_component(FB ${FBS} NAME_WE)

  add_custom_command(
    OUTPUT ${FBS_GENERATED_DIR}/${FB}_generated.h
    DEPENDS ${FBS}
    COMMAND flatbuffers::flatc --cpp --no-includes ${FBS}
    WORKING_DIRECTORY ${FBS_GENERATED_DIR}
    )

  add_custom_command(
    OUTPUT ${FB}.bfbs
    DEPENDS ${FBS}
    COMMAND flatbuffers::flatc -b --schema ${FBS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )

  add_custom_command(
    OUTPUT ${FB}.bfbs.o
    DEPENDS ${FB}.bfbs
    COMMAND ld -r -b binary ${FB}.bfbs -o ${FB}.bfbs.o
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endforeach()


# ===== Generate common fbs spec with enums =====
foreach(FBS ${SCHEMAS_COMMON})
    get_filename_component(FB ${FBS} NAME_WE)
    add_custom_command(
        OUTPUT ${FBS_GENERATED_COMMON_DIR}/${FB}_generated.h
        DEPENDS ${FBS}
        COMMAND flatbuffers::flatc --cpp --no-includes ${FBS}
        WORKING_DIRECTORY ${FBS_GENERATED_COMMON_DIR}
        )
endforeach()

# ==== Libraries ====
add_library(ouster_osf STATIC src/frame.cpp
                              src/chunk.cpp
                              src/stream.cpp
                              src/util.cpp
                              src/operations.cpp
                              src/file.cpp
                              src/file_info.cpp
                              src/message.cpp
                              src/reader.cpp
                              src/png_tools.cpp
                              src/lidar_scan_stat.cpp
                              ${FBS_GENERATED_COMMON_DIR}/osfCommon_generated.h
                              ${FBS_GENERATED_DIR}/osfChunk_generated.h
                              ${FBS_GENERATED_DIR}/osfSession_generated.h
                              ${FBS_GENERATED_DIR}/osfHeader_generated.h)
target_link_libraries(ouster_osf
  PUBLIC jsoncpp ${PNG_LIBRARIES} yaml-cpp ouster_client PRIVATE flatbuffers::flatbuffers)
target_include_directories(ouster_osf 
PUBLIC include
SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${FBS_GENERATED_COMMON_DIR}
PRIVATE ${jsoncpp_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${FBS_GENERATED_DIR})

# ==== Executables ====
add_executable(streamParser src/parse_stream.cpp)
target_link_libraries(streamParser ouster_osf ouster_client)
target_include_directories(streamParser PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_executable(frameChunker src/chunk_frames.cpp)
target_link_libraries(frameChunker ouster_osf ouster_client)
target_include_directories(frameChunker PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${FBS_GENERATED_DIR})

add_executable(stream2osf src/stream2osf.cpp)
target_link_libraries(stream2osf ouster_osf)
target_include_directories(stream2osf PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_executable(osf-metadata src/dump_metadata.cpp
  osfSession.bfbs.o osfHeader.bfbs.o)
target_link_libraries(osf-metadata ouster_osf)
target_include_directories(osf-metadata PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

add_executable(osfExtractor src/extract_timerange.cpp)
target_link_libraries(osfExtractor ouster_osf)
target_include_directories(osfExtractor PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${FBS_GENERATED_DIR})

# ====== Tests ====

# Each test file should be in a format "<test_name>_test.cpp"
set(OSF_TESTS_SOURCES test/util_test.cpp
                      test/stream_test.cpp
                      test/file_test.cpp
                      test/reader_test.cpp
                      test/file_info_test.cpp
                      test/frame_chunker_test.cpp
                      )


enable_testing()

# Create "osf_<test_name>" tests for every test
foreach(TEST_FULL_NAME ${OSF_TESTS_SOURCES})
  get_filename_component(TEST_FILENAME ${TEST_FULL_NAME} NAME_WE)
  add_executable(osf_${TEST_FILENAME} ${TEST_FULL_NAME})
  # Hack: only specific internal tests to have access to _generated.h headers
  if(${TEST_FILENAME} MATCHES "stream_test|util_test|frame_chunker_test")
    target_include_directories(osf_${TEST_FILENAME} PRIVATE src ${FBS_GENERATED_DIR})
  endif()
  target_link_libraries(osf_${TEST_FILENAME} ouster_osf gtest gtest_main pthread)
  add_test(NAME osf_${TEST_FILENAME}
           COMMAND osf_${TEST_FILENAME} --gtest_output=xml:osf_${TEST_FILENAME}.xml)
endforeach()

# ==== Installs ========================
install(TARGETS osfExtractor osf-metadata stream2osf frameChunker
  DESTINATION bin)
