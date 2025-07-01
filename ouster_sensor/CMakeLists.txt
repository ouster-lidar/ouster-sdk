# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(CURL REQUIRED)
find_package(Threads REQUIRED)

include(Coverage)

# ==== Libraries ====
add_library(ouster_sensor STATIC src/client.cpp src/netcompat.cpp src/sensor_packet_source.cpp
  src/sensor_http.cpp src/sensor_http_imp.cpp src/sensor_scan_source.cpp
  src/sensor_tcp_imp.cpp)
target_link_libraries(ouster_sensor
  PUBLIC
    OusterSDK::ouster_client
    Eigen3::Eigen
    $<BUILD_INTERFACE:ouster_build>
    Threads::Threads
    $<INSTALL_INTERFACE:CURL::libcurl>
  PRIVATE
    $<BUILD_INTERFACE:CURL::libcurl>
    )
target_compile_definitions(ouster_sensor PRIVATE EIGEN_MPL2_ONLY)
if(BUILD_SHARED_LIBRARY)
  target_compile_definitions(ouster_sensor PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()
set_property(TARGET ouster_sensor PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_sensor PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
CodeCoverageFunctionality(ouster_sensor)

add_library(OusterSDK::ouster_sensor ALIAS ouster_sensor)

if(WIN32)
  target_link_libraries(ouster_sensor PUBLIC ws2_32)
endif()

target_include_directories(ouster_sensor 
  PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(ouster_sensor SYSTEM 
  PUBLIC
  PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty>)

# ==== Install ====
install(TARGETS ouster_sensor
        EXPORT ouster-sdk-targets
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster
  DESTINATION include
)
