cmake_minimum_required(VERSION 2.8.3)
project(ouster_ros)

add_compile_options(-std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  roscpp
  ouster_client
)

add_message_files(DIRECTORY msg FILES PacketMsg.msg)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ouster_ros
  CATKIN_DEPENDS pcl_ros pcl_conversions roscpp message_runtime std_msgs sensor_msgs
)

add_library(ouster_ros STATIC src/os1_ros.cpp)
target_link_libraries(ouster_ros ${catkin_LIBRARIES})
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

add_executable(os1_node src/os1_node.cpp)
target_link_libraries(os1_node ouster_ros ${catkin_LIBRARIES})
