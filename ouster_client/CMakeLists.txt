cmake_minimum_required(VERSION 2.8.3)
project(ouster_client)

add_compile_options(-std=c++11 -Wall)

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


include_directories(include)
add_library(ouster_client STATIC src/os1.cpp)
add_executable(ouster_client_example src/main.cpp)
target_link_libraries(ouster_client_example ouster_client)
