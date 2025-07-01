# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

include(Coverage)

# ==== Libraries ====
add_library(ouster_client STATIC src/types.cpp src/sensor_info.cpp src/lidar_scan.cpp
  src/image_processing.cpp src/parsing.cpp src/packet_source.cpp src/compat_ops.cpp
  src/logging.cpp src/field.cpp src/profile_extension.cpp src/metadata.cpp src/packet.cpp
  src/open_source.cpp src/scan_source.cpp src/scan_source_utils.cpp src/io_type.cpp
  src/transform_quaternion.cpp
  src/transform_vector.cpp
  src/transform_homogeneous.cpp
  src/pose_util.cpp
  src/pose_conversion.cpp
  src/cloud_io.cpp
  src/error_handler.cpp
)
target_link_libraries(ouster_client
  PUBLIC
    $<BUILD_INTERFACE:ouster_build>
    Threads::Threads
    $<BUILD_INTERFACE:Eigen3::Eigen>
    )
target_compile_definitions(ouster_client PRIVATE EIGEN_MPL2_ONLY)
if(BUILD_SHARED_LIBRARY)
  target_compile_definitions(ouster_client PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()
set_property(TARGET ouster_client PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_client PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
CodeCoverageFunctionality(ouster_client)

add_library(OusterSDK::ouster_client ALIAS ouster_client)

if(WIN32)
  target_link_libraries(ouster_client PUBLIC ws2_32)
endif()

target_include_directories(ouster_client
  PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(ouster_client SYSTEM
  PUBLIC
      $<INSTALL_INTERFACE:include/optional-lite>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/optional-lite>
  PRIVATE
      $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>)

# ==== Install ====
install(TARGETS ouster_client
        EXPORT ouster-sdk-targets
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster include/optional-lite
  DESTINATION include
)
