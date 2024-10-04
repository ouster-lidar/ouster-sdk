# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)
find_package(spdlog REQUIRED)
include(Coverage)

# ==== Libraries ====
add_library(ouster_client src/client.cpp src/types.cpp src/sensor_info.cpp src/netcompat.cpp src/lidar_scan.cpp
  src/image_processing.cpp src/parsing.cpp src/sensor_client.cpp
  src/sensor_http.cpp src/sensor_http_imp.cpp src/sensor_scan_source.cpp
  src/sensor_tcp_imp.cpp src/logging.cpp src/field.cpp src/profile_extension.cpp src/util.cpp src/metadata.cpp)
target_link_libraries(ouster_client
  PUBLIC
    Eigen3::Eigen
    jsoncpp_lib
    $<BUILD_INTERFACE:ouster_build>
    spdlog::spdlog
  PRIVATE
    CURL::libcurl)
target_compile_definitions(ouster_client PRIVATE EIGEN_MPL2_ONLY)
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
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>)

# ==== Install ====
install(TARGETS ouster_client
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster include/optional-lite DESTINATION include)
