cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_client VERSION 0.1.0)
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
target_include_directories(ouster_build INTERFACE ${CMAKE_CURRENT_BINARY_DIR}/include)
add_dependencies(ouster_build generate_build_header)

# ==== Libraries ====
add_library(ouster_client src/client.cpp src/types.cpp src/netcompat.cpp src/lidar_scan.cpp)
target_link_libraries(ouster_client PUBLIC jsoncpp_lib Eigen3::Eigen ouster_build)
if(WIN32)
  target_link_libraries(ouster_client PUBLIC ws2_32)
endif()
target_include_directories(ouster_client PUBLIC include)

# ==== Executables ====
add_executable(ouster_client_example src/example.cpp)
target_link_libraries(ouster_client_example ouster_client ouster_build)
