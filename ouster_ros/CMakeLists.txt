cmake_minimum_required(VERSION 3.1.0)
project(ouster_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS         "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  roscpp
  ouster_client
  ouster_viz
  tf2
  tf2_ros
)

add_message_files(FILES PacketMsg.msg)
add_service_files(FILES OS1ConfigSrv.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ouster_ros
  CATKIN_DEPENDS
    roscpp message_runtime pcl_ros
    std_msgs sensor_msgs geometry_msgs
    ouster_client ouster_viz
)


if (CATKIN_ENABLE_TESTING)

#  find_package(rostest REQUIRED)
#  add_rostest(tests/os1_replay.test)

#  catkin_download_test_data(
#	${PROJECT_NAME}_test.bag
#	https://data.ouster.io/sample-data-2018-08-29/2018-08-29-16-46-17_2.bag
#	DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
#	MD5 78ef679d4aede50a6725ad303de63d75)

endif()

add_library(ouster_ros STATIC src/os1_ros.cpp)
target_link_libraries(ouster_ros ${catkin_LIBRARIES})
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

#add_executable(pc2_test src/pc2_test.cpp)
#target_link_libraries(pc2_test ${catkin_LIBRARIES})

add_executable(os1_node src/os1_node.cpp)
target_link_libraries(os1_node ouster_ros ${catkin_LIBRARIES})

add_executable(os1_cloud_node src/os1_cloud_node.cpp)
target_link_libraries(os1_cloud_node ouster_ros ${catkin_LIBRARIES})

add_executable(viz_node src/viz_node.cpp)
target_link_libraries(viz_node ${catkin_LIBRARIES})

add_executable(img_node src/img_node.cpp)
target_link_libraries(img_node ouster_ros ${catkin_LIBRARIES})
