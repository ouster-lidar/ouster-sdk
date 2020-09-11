cmake_minimum_required(VERSION 3.1.0)

project(shared_sw)
include (CTest)
enable_testing()

# ============ Public ============
get_filename_component(ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/ouster_example" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH ${ABSOLUTE_PATH})
get_filename_component(ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH ${ABSOLUTE_PATH})
find_package(ouster_client REQUIRED)
find_package(ouster_viz REQUIRED)
find_package(ouster_pcap REQUIRED)

if(WIN32)
  SET(ZIP_EXTENSION "zip")
  SET(VIZ_PATH "${CMAKE_CURRENT_BINARY_DIR}/ouster_viz/Release/*")
else()
  SET(ZIP_EXTENSION "tar.gz")
  SET(VIZ_PATH "${CMAKE_CURRENT_BINARY_DIR}/ouster_viz/simple_viz")
endif()

# ============ Source Zip ============
add_custom_target(srczip)
add_custom_command(TARGET srczip
  COMMAND git archive --format ${ZIP_EXTENSION} HEAD:ouster_example -o ${CMAKE_CURRENT_BINARY_DIR}/ouster_example.${ZIP_EXTENSION}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  
# ============ Binary Zip ============
add_custom_target(binzip DEPENDS ouster_client ouster_client_example simple_viz)
add_custom_command(TARGET binzip
  COMMAND cmake -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/binzip/
  )

# Copy the files to the temp directory
if(WIN32)
  # Sigh, windows path stupidities
  file(TO_NATIVE_PATH "${VIZ_PATH}" VIZ_PATH )
  file(TO_NATIVE_PATH "${CMAKE_CURRENT_BINARY_DIR}" BIN_PATH )
  add_custom_command(TARGET binzip
    COMMAND xcopy ${VIZ_PATH} ${BIN_PATH}\\binzip /Y
    )
else()
  add_custom_command(TARGET binzip
    COMMAND cmake -E copy ${VIZ_PATH} ${CMAKE_CURRENT_BINARY_DIR}/binzip
    )
endif()

add_custom_command(TARGET binzip
  COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ouster_example/LICENSE ${CMAKE_CURRENT_BINARY_DIR}/binzip
  COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ouster_example/CHANGELOG.md ${CMAKE_CURRENT_BINARY_DIR}/binzip
  COMMAND cmake -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ouster_example/ouster_viz/README.md ${CMAKE_CURRENT_BINARY_DIR}/binzip
  )
if(WIN32)
  add_custom_command(TARGET binzip
    COMMAND cmake -E tar c ${CMAKE_CURRENT_BINARY_DIR}/ouster_example_bin.${ZIP_EXTENSION} --format=zip .
    WORKING_DIRECTORY binzip
    )
else()
  add_custom_command(TARGET binzip
    COMMAND cmake -E tar c ${CMAKE_CURRENT_BINARY_DIR}/ouster_example_bin.${ZIP_EXTENSION} .
    WORKING_DIRECTORY binzip
    )
endif()
add_custom_command(TARGET binzip
  COMMAND cmake -E remove_directory ${CMAKE_CURRENT_BINARY_DIR}/binzip
  )

# ============ Internal ============
get_filename_component(ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH ${ABSOLUTE_PATH})

