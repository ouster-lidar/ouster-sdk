cmake_minimum_required(VERSION 2.8.3)
project(ouster_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes rviz urdf worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/os1_gazebo.test)
endif()
