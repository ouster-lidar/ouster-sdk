cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_client VERSION 0.4.1)
set(ouster_client_VERSION_SUFFIX "")

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

# ==== Build Metadata Target ====
add_custom_target(generate_build_header)
add_custom_command(TARGET generate_build_header PRE_BUILD
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
add_dependencies(ouster_build generate_build_header)

# ==== Libraries ====
add_library(ouster_client src/client.cpp src/types.cpp src/netcompat.cpp src/lidar_scan.cpp
  src/image_processing.cpp src/buffered_udp_source.cpp src/parsing.cpp)
target_link_libraries(ouster_client
  PUBLIC Eigen3::Eigen $<BUILD_INTERFACE:ouster_build>
  PRIVATE jsoncpp_lib)
target_compile_definitions(ouster_client PRIVATE EIGEN_MPL2_ONLY)
add_library(OusterSDK::ouster_client ALIAS ouster_client)

if(WIN32)
  target_link_libraries(ouster_client PUBLIC ws2_32)
endif()
target_include_directories(ouster_client PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(ouster_client SYSTEM PUBLIC
  $<INSTALL_INTERFACE:include/optional-lite>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/optional-lite>)

# ==== Executables ====
add_executable(ouster_client_example src/example.cpp)
target_link_libraries(ouster_client_example PRIVATE ouster_client ouster_build)
add_executable(OusterSDK::ouster_client_example ALIAS ouster_client_example)

# ==== Install ====
install(TARGETS ouster_client ouster_client_example
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster include/optional-lite DESTINATION include)
