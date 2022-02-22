cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_viz)

# ==== Requirements ====
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(Threads)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

find_path(TCLAP_INCLUDE_DIRS "tclap/Arg.h")
if(TCLAP_INCLUDE_DIRS)
  message(STATUS "Found tclap: ${TCLAP_INCLUDE_DIRS}")
else()
  message(STATUS "Warning: tclap NOT found")
  set(TCLAP_INCLUDE_DIRS "")
endif()

# ==== Libraries ====
# use only MPL-licensed parts of eigen
add_definitions(-DEIGEN_MPL2_ONLY)

add_library(ouster_viz src/point_viz.cpp src/cloud.cpp src/camera.cpp src/image.cpp
  src/lidar_scan_viz.cpp src/gltext.cpp src/misc.cpp src/glfw.cpp)
target_link_libraries(ouster_viz
  PRIVATE Eigen3::Eigen glfw GLEW::GLEW OpenGL::GL ouster_client)
target_include_directories(ouster_viz PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
add_library(OusterSDK::ouster_viz ALIAS ouster_viz)

# ==== Executables ====
add_executable(simple_viz src/simple_viz.cpp)
target_link_libraries(simple_viz Threads::Threads ouster_client ouster_viz)
target_include_directories(simple_viz PRIVATE ${TCLAP_INCLUDE_DIRS})
add_executable(OusterSDK::simple_viz ALIAS simple_viz)

# ==== Install ====
install(TARGETS ouster_viz simple_viz
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
