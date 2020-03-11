cmake_minimum_required(VERSION 3.1.0)
project(ouster_client)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)

if (DEFINED CATKIN_DEVEL_PREFIX)
  find_package(catkin REQUIRED)
  message(STATUS "Catkin env set, compiling ${PROJECT_NAME} with ROS")

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ouster_client
    DEPENDS jsoncpp
    CATKIN_DEPENDS
  )
else()
  message(STATUS "Catkin env not set, compiling ${PROJECT_NAME} without ROS")
endif()

add_library(ouster_client STATIC
  src/os1.cpp
  src/os1_util.cpp)
set_target_properties(ouster_client PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ouster_client jsoncpp)
target_include_directories(ouster_client PUBLIC include)
target_include_directories(ouster_client SYSTEM PRIVATE ${jsoncpp_INCLUDE_DIRS})

add_executable(ouster_client_example src/main.cpp)
target_link_libraries(ouster_client_example ouster_client)
