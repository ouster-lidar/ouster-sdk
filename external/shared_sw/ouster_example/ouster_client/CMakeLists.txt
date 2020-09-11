cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_client)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/version.cpp ALL
  COMMAND ${CMAKE_COMMAND} -DVERSION_GEN_BUILD_DIR="${CMAKE_CURRENT_BINARY_DIR}"
                           -DVERSION_GEN_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ousterVersionGen.cmake)

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
if(WIN32)
  find_package(jsoncpp CONFIG REQUIRED)
  set(OUSTER_CLIENT_ADDITIONAL_LIBS jsoncpp_lib ws2_32)
  add_definitions(-D_USE_MATH_DEFINES)
  add_compile_options(/W4)
elseif(APPLE)
  # jsoncpp from brew currently doesn't have a cmake config
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(jsoncpp REQUIRED jsoncpp)
  find_library(OUSTER_CLIENT_ADDITIONAL_LIBS NAMES ${jsoncpp_LIBRARIES}
                                             HINTS ${jsoncpp_LIBRARY_DIRS})
  add_compile_options(-Wall -Wextra -Werror)
else()
  find_package(jsoncpp REQUIRED)
  set(OUSTER_CLIENT_ADDITIONAL_LIBS jsoncpp_lib)
  add_compile_options(-Wall -Wextra -Werror)
endif()


# ==== Options ====
set(CMAKE_CXX_STANDARD 11)

# ==== Libraries ====
add_library(ouster_client_version ${CMAKE_CURRENT_BINARY_DIR}/version.cpp)
set_target_properties(ouster_client_version PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(ouster_client STATIC src/client.cpp src/types.cpp src/compat.cpp
                                 src/lidar_scan.cpp)
set_target_properties(ouster_client PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ouster_client PUBLIC ouster_client_version
                                    ${OUSTER_CLIENT_ADDITIONAL_LIBS})
target_include_directories(ouster_client PUBLIC include)
target_include_directories(ouster_client SYSTEM PUBLIC ${jsoncpp_INCLUDE_DIRS}
                                         ${EIGEN3_INCLUDE_DIR})

# ==== Executables ====
add_executable(ouster_client_example src/main.cpp)
target_link_libraries(ouster_client_example ouster_client ouster_client_version)
target_include_directories(ouster_client_example PRIVATE include ${EIGEN3_INCLUDE_DIR})
