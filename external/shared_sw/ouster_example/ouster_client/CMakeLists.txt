cmake_minimum_required(VERSION 3.1.0)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

# ==== Project Name ====
project(ouster_client)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.cpp ALL
  COMMAND ${CMAKE_COMMAND} -DVERSION_GEN_BUILD_DIR="${CMAKE_CURRENT_BINARY_DIR}"
                           -DVERSION_GEN_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ousterVersionGen.cmake)

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)

if(WIN32)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

# ==== Libraries ====
add_library(ouster_client_version ${CMAKE_CURRENT_BINARY_DIR}/version.cpp)
set_target_properties(ouster_client_version PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(ouster_client STATIC src/client.cpp src/types.cpp src/compat.cpp src/lidar_scan.cpp)
set_target_properties(ouster_client PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ouster_client PUBLIC include)
target_link_libraries(ouster_client PUBLIC ouster_client_version jsoncpp_lib Eigen3::Eigen)
if(WIN32)
  target_link_libraries(ouster_client PUBLIC ws2_32)
endif()

# ==== Executables ====
add_executable(ouster_client_example src/main.cpp)
target_link_libraries(ouster_client_example ouster_client ouster_client_version)
