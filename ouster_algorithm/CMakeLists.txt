cmake_minimum_required(VERSION 3.16.3)

find_package(Eigen3 REQUIRED)

add_library(ouster_algorithm STATIC
  src/align_clouds.cpp
  src/ground_seg.cpp
  src/normals.cpp
  src/voxel_downsample.cpp
)
set_target_properties(ouster_algorithm PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
)

target_include_directories(ouster_algorithm
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(ouster_algorithm
  PUBLIC
    Eigen3::Eigen
    OusterSDK::ouster_core
)

add_library(OusterSDK::ouster_algorithm ALIAS ouster_algorithm)

set_property(TARGET ouster_algorithm PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_algorithm PROPERTIES CXX_VISIBILITY_PRESET hidden)
  target_compile_definitions(ouster_algorithm PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()

install(TARGETS ouster_algorithm
  EXPORT ouster-sdk-targets
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ouster DESTINATION include)
