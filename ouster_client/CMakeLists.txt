# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(nmea REQUIRED)
find_package(libzip REQUIRED)
find_package(OpenSSL REQUIRED)

include(Coverage)

# ==== Libraries ====
add_library(ouster_client STATIC src/types.cpp src/sensor_info.cpp src/lidar_scan.cpp
  src/image_processing.cpp src/parsing.cpp src/packet_source.cpp src/compat_ops.cpp
  src/logging.cpp src/field.cpp src/profile_extension.cpp src/metadata.cpp src/packet.cpp
  src/open_source.cpp src/scan_source.cpp src/scan_source_utils.cpp src/io_type.cpp
  src/multi_scan_source.cpp
  src/transform_quaternion.cpp
  src/transform_vector.cpp
  src/transform_homogeneous.cpp
  src/pose_util.cpp
  src/pose_conversion.cpp
  src/cloud_io.cpp
  src/error_handler.cpp
  src/zone_monitor.cpp
  src/lidar_scan_set.cpp
  src/json_tools.cpp
  src/normals.cpp
  src/zip.cpp
  src/chanfield.cpp
  src/xyzlut.cpp
  src/mesh.cpp
  src/triangle.cpp
  src/stl.cpp
  src/zone.cpp
  src/zrb.cpp
  src/zone_header.cpp
  src/vector_streambuf.cpp
  src/sha256.cpp
  src/beam_config.cpp
  src/typedefs.cpp
)
target_link_libraries(ouster_client
  PUBLIC
    Eigen3::Eigen
    $<BUILD_INTERFACE:ouster_build>
    Threads::Threads
  PRIVATE
    nmea
    libzip::zip
    ${OPENSSL_LIBRARIES}
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
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
      ${OPENSSL_INCLUDE_DIR}
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/spdlog>
)

target_include_directories(ouster_client SYSTEM
  PUBLIC
      $<INSTALL_INTERFACE:include/optional-lite>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/optional-lite>
      $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>)

# ==== Install ====
install(TARGETS ouster_client nmea
        EXPORT ouster-sdk-targets
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster include/optional-lite
  DESTINATION include
)

add_executable(zone_render_mini src/zone_render_mini.cpp)
target_link_libraries(zone_render_mini PRIVATE ouster_client)
target_include_directories(zone_render_mini SYSTEM
  PUBLIC
      $<INSTALL_INTERFACE:include/optional-lite>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/optional-lite>
      $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
  PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>)
