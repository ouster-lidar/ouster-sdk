project(ouster_client VERSION 0.4.1.3)
set(ouster_client_VERSION_SUFFIX "")

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)

# ==== Build Metadata Target ====
add_custom_target(gen_header)
add_custom_command(TARGET gen_header PRE_BUILD
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND ${CMAKE_COMMAND} -DVERSION_GEN_OUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/include/ouster"
                           -DVERSION_GEN_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
                           -DBUILD_TYPE="${CMAKE_BUILD_TYPE}"
                           -DBUILD_SYSTEM="${CMAKE_SYSTEM_NAME}"
                           -Douster_client_VERSION="${ouster_client_VERSION}"
                           -Douster_client_VERSION_SUFFIX="${ouster_client_VERSION_SUFFIX}"
  -P ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/VersionGen.cmake)

add_library(ouster_build INTERFACE)
target_include_directories(ouster_build INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)
add_dependencies(ouster_build gen_header)

# ==== Libraries ====
add_library(ouster_client src/client.cpp src/types.cpp src/netcompat.cpp src/lidar_scan.cpp
  src/image_processing.cpp src/buffered_udp_source.cpp src/parsing.cpp
  src/sensor_http.cpp src/sensor_http_imp.cpp src/sensor_tcp_imp.cpp)
target_link_libraries(ouster_client
  PUBLIC
    Eigen3::Eigen
    $<BUILD_INTERFACE:ouster_build>
  PRIVATE
    CURL::libcurl
    jsoncpp_lib)
target_compile_definitions(ouster_client PRIVATE EIGEN_MPL2_ONLY)
add_library(OusterSDK::ouster_client ALIAS ouster_client)

# If ouster_client is build as >=c++17, the nonstd::optional backport
# will just be an alias for std::optional. In that case, client code
# must also build as c++17 to use the same implementation of optional
get_target_property(OUSTER_CLIENT_CXX_STANDARD ouster_client CXX_STANDARD)
if(OUSTER_CLIENT_CXX_STANDARD GREATER_EQUAL 17)
  target_compile_features(ouster_client INTERFACE cxx_std_17)
endif()

if(WIN32)
  target_link_libraries(ouster_client PUBLIC ws2_32)
endif()
target_include_directories(ouster_client PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(ouster_client SYSTEM PUBLIC
  $<INSTALL_INTERFACE:include/optional-lite>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/optional-lite>)

# ==== Install ====
install(TARGETS ouster_client
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster include/optional-lite DESTINATION include)
