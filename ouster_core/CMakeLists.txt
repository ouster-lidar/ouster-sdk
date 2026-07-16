# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(nmea REQUIRED)
find_package(libzip REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(RobinMap REQUIRED MODULE)

include(Coverage)

# ==== Libraries ====
add_library(ouster_core STATIC src/types.cpp src/sensor_info.cpp src/data_format.cpp src/sensor_config.cpp src/lidar_frame.cpp
  src/image_processing.cpp src/parsing.cpp src/packet_source.cpp src/compat_ops.cpp
  src/logging.cpp src/field.cpp src/profile_extension.cpp src/metadata.cpp src/packet.cpp
  src/open_source.cpp src/frame_set_source.cpp src/frame_set_source_utils.cpp src/io_type.cpp
  src/multi_frame_set_source.cpp
  src/frame_ops.cpp
  src/transform_quaternion.cpp
  src/transform_vector.cpp
  src/transform_homogeneous.cpp
  src/pose_util.cpp
  src/pose.cpp
  src/pose_conversion.cpp
  src/cloud_io.cpp
  src/voxel_hash_map.cpp
  src/error_handler.cpp
  src/zone_monitor.cpp
  src/frame_set.cpp
  src/json_tools.cpp
  src/zip.cpp
  src/chanfield.cpp
  src/xyzlut.cpp
  src/mesh.cpp
  src/kmp.cpp
  src/triangle.cpp
  src/stl.cpp
  src/zone.cpp
  src/zone_state.cpp
  src/zrb.cpp
  src/zone_header.cpp
  src/vector_streambuf.cpp
  src/sha256.cpp
  src/beam_config.cpp
  src/typedefs.cpp
  src/object.cpp
  src/object_util.cpp
)
target_link_libraries(ouster_core
  PUBLIC
    Eigen3::Eigen
    $<BUILD_INTERFACE:ouster_build>
    Threads::Threads
    $<BUILD_INTERFACE:tsl::robin_map>
  PRIVATE
    nmea
    ouster_build
    libzip::zip
    ${OPENSSL_LIBRARIES}
    )
target_compile_definitions(ouster_core PRIVATE EIGEN_MPL2_ONLY)
if(BUILD_SHARED_LIBRARY)
  target_compile_definitions(ouster_core PRIVATE BUILD_SHARED_LIBS_EXPORT)
  target_compile_definitions(ouster_build PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()
set_property(TARGET ouster_core PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_core PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
CodeCoverageFunctionality(ouster_core)

add_library(OusterSDK::ouster_core ALIAS ouster_core)
add_library(ouster_client ALIAS ouster_core)
add_library(OusterSDK::ouster_client ALIAS ouster_core)

if(WIN32)
  target_link_libraries(ouster_core PUBLIC ws2_32)
endif()

target_include_directories(ouster_core
  PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
      ${OPENSSL_INCLUDE_DIR}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/spdlog>
)

target_include_directories(ouster_core SYSTEM
  PUBLIC
      $<INSTALL_INTERFACE:include/optional-lite>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/optional-lite>
      $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>)

# ==== Install ====
install(TARGETS ouster_core nmea ouster_build
        EXPORT ouster-sdk-targets
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster include/optional-lite
  DESTINATION include
)

# voxel_hash_map.h is a public header and includes <tsl/robin_map.h>.
# Install the vendored robin-map headers so consumers of the installed SDK
# (e.g. Python CLI prebuilt build) can find them without needing the source tree.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/robin-map/include/tsl
  DESTINATION include
)

add_executable(zone_render_mini src/zone_render_mini.cpp)
target_link_libraries(zone_render_mini PRIVATE ouster_core)
target_include_directories(zone_render_mini SYSTEM
  PUBLIC
      $<INSTALL_INTERFACE:include/optional-lite>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/optional-lite>
      $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>)
