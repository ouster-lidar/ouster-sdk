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



if (CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest(tests/os1_replay.test)

  catkin_download_test_data(
	${PROJECT_NAME}_test.bag
	https://data.ouster.io/sample-data-2018-08-29/2018-08-29-16-46-17_2.bag
	DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
	MD5 78ef679d4aede50a6725ad303de63d75)

endif()

add_library(ouster_ros STATIC src/os1_ros.cpp)
target_link_libraries(ouster_ros ${catkin_LIBRARIES})
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

add_executable(pc2_test src/pc2_test.cpp)
target_link_libraries(pc2_test ${catkin_LIBRARIES})
