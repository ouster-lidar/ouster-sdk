cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             pcl_ros
             pcl_conversions
             roscpp
             tf2
             tf2_ros
             tf2_geometry_msgs)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(ouster_client REQUIRED)
find_package(ouster_viz REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Werror)

# ==== Catkin ====
add_message_files(FILES PacketMsg.msg)
add_service_files(FILES OSConfigSrv.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ouster_ros
  CATKIN_DEPENDS
  roscpp
  message_runtime
  pcl_ros
  std_msgs
  sensor_msgs
  geometry_msgs)

# ==== Libraries ====
add_library(ouster_ros STATIC src/ros.cpp)
target_link_libraries(
  ouster_ros
  PUBLIC ${catkin_LIBRARIES}
  PRIVATE ouster_client)
set_target_properties(ouster_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ouster_ros PUBLIC include ${catkin_INCLUDE_DIRS})
target_compile_options(ouster_ros PRIVATE "$<$<CONFIG:DEBUG>:${FLAGS_DEBUG}>")
target_compile_options(ouster_ros
                       PRIVATE "$<$<CONFIG:RELEASE>:${FLAGS_RELEASE}>")
target_compile_options(ouster_ros PRIVATE "${FLAGS}")
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

# ==== Executables ====
add_executable(os_node src/os_node.cpp)
target_link_libraries(os_node ouster_ros ${catkin_LIBRARIES} ouster_viz
                      ouster_client)
target_include_directories(os_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(os_node ${PROJECT_NAME}_gencpp)

add_executable(os_cloud_node src/os_cloud_node.cpp)
target_link_libraries(os_cloud_node ouster_ros ${catkin_LIBRARIES} ouster_viz
                      ouster_client ${VTK_LIBRARIES})
target_include_directories(os_cloud_node PRIVATE include
                           ${catkin_INCLUDE_DIRS})
add_dependencies(os_cloud_node ${PROJECT_NAME}_gencpp)

add_executable(viz_node src/viz_node.cpp)
target_link_libraries(viz_node ${catkin_LIBRARIES} ouster_viz ouster_client
                      glfw GLEW)
target_include_directories(viz_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(viz_node ${PROJECT_NAME}_gencpp)

add_executable(img_node src/img_node.cpp)
target_link_libraries(img_node ouster_ros ${catkin_LIBRARIES} ouster_viz
                      ouster_client ${VTK_LIBRARIES})
target_include_directories(img_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(img_node ${PROJECT_NAME}_gencpp)

# ==== Install ====
install(
  TARGETS os_node os_cloud_node viz_node img_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES ouster.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
