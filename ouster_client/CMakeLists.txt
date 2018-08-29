cmake_minimum_required(VERSION 3.1.0)
project(ouster_client)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS         "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(catkin QUIET)
if (catkin_FOUND)
  message(STATUS "Catkin found, compiling with ROS")

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ouster_client
    CATKIN_DEPENDS
  )
else()
  message(STATUS "Catkin not found, compiling without ROS")
endif()

add_library(ouster_client STATIC src/os1.cpp)
target_include_directories(ouster_client PUBLIC include)

add_executable(ouster_client_example src/main.cpp)
target_link_libraries(ouster_client_example ouster_client)
