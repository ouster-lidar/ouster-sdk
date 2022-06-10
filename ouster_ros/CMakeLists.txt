cmake_minimum_required(VERSION 3.1.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_ros)

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(tf2_eigen REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             std_msgs
             sensor_msgs
             geometry_msgs
             pcl_conversions
             roscpp
             tf2
             tf2_ros
             nodelet)

# ==== Options ====
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Werror -Wno-error=deprecated-declarations)
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

# ==== Catkin ====
add_message_files(FILES PacketMsg.msg)
add_service_files(FILES GetConfig.srv SetConfig.srv GetMetadata.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

set(_ouster_ros_INCLUDE_DIRS
  "include;../ouster_client/include;../ouster_client/include/optional-lite")

catkin_package(
  INCLUDE_DIRS
    ${_ouster_ros_INCLUDE_DIRS}
  LIBRARIES
    ouster_ros
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
  DEPENDS
    EIGEN3)

catkin_package(
  LIBRARIES nodelet_os_cloud
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

catkin_package(
  LIBRARIES nodelet_os_image
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

# ==== Libraries ====
# Build static libraries and bundle them into ouster_ros using the `--whole-archive` flag. This is
# necessary because catkin doesn't interoperate easily with target-based cmake builds. Object
# libraries are the recommended way to do this, but require >=3.13 to propagate usage requirements.
set(_SAVE_BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
add_subdirectory(../ouster_client ouster_client EXCLUDE_FROM_ALL)
target_compile_options(ouster_client PRIVATE -Wno-deprecated-declarations)
message(STATUS "Ouster SDK client: Using EIGEN_MAX_ALIGN_BYTES = 32")
target_compile_definitions(ouster_client PUBLIC EIGEN_MAX_ALIGN_BYTES=32)
set(BUILD_SHARED_LIBS ${_SAVE_BUILD_SHARED_LIBS})

# catkin adds all include dirs to a single variable, don't try to use targets
include_directories(${_ouster_ros_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# use only MPL-licensed parts of eigen
add_definitions(-DEIGEN_MPL2_ONLY)

add_library(ouster_ros src/ros.cpp)
target_link_libraries(ouster_ros PUBLIC ${catkin_LIBRARIES} ouster_build pcl_common PRIVATE
  -Wl,--whole-archive ouster_client -Wl,--no-whole-archive)
target_compile_definitions(ouster_ros PUBLIC EIGEN_MAX_ALIGN_BYTES=32)
add_dependencies(ouster_ros ${PROJECT_NAME}_gencpp)

# ==== Executables ====
add_library(nodelets_os
  src/os_client_base_nodelet.cpp
  src/os_sensor_nodelet.cpp
  src/os_replay_nodelet.cpp
  src/os_cloud_nodelet.cpp
  src/os_image_nodelet.cpp)
target_link_libraries(nodelets_os ouster_ros ${catkin_LIBRARIES})
add_dependencies(nodelets_os ${PROJECT_NAME}_gencpp)

# ==== Install ====
install(
  TARGETS
    ouster_ros
    nodelets_os
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(
  DIRECTORY ${_ouster_ros_INCLUDE_DIRS}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(
  FILES
    ../LICENSE
    ../LICENSE-bin
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(
    DIRECTORY
      launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(
  DIRECTORY
    config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
