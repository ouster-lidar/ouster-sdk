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

add_executable(${PROJECT_NAME} src/os1_node.cpp src/os1_ros.cpp src/os1.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

