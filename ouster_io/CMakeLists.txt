cmake_minimum_required(VERSION 3.16.3)

find_package(PNG REQUIRED)

add_library(ouster_io STATIC
  src/image_io.cpp
)
set_target_properties(ouster_io PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

target_include_directories(ouster_io
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_include_directories(ouster_io SYSTEM
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>
)

target_link_libraries(ouster_io
  PUBLIC
    OusterSDK::ouster_core
  PRIVATE
    PNG::PNG
)

add_library(OusterSDK::ouster_io ALIAS ouster_io)

set_property(TARGET ouster_io PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_io PROPERTIES CXX_VISIBILITY_PRESET hidden)
  target_compile_definitions(ouster_io PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()

install(TARGETS ouster_io
  EXPORT ouster-sdk-targets
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ouster DESTINATION include)
