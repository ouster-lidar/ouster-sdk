cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_viz)

# ==== Requirements ====
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(Threads)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

# ==== Libraries ====
add_library(ouster_viz src/point_viz.cpp src/cloud.cpp src/camera.cpp src/image.cpp
  src/lidar_scan_viz.cpp)
target_link_libraries(ouster_viz
  PUBLIC Threads::Threads glfw GLEW::GLEW OpenGL::GL
  PRIVATE ouster_client)
target_include_directories(ouster_viz PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
add_library(OusterSDK::ouster_viz ALIAS ouster_viz)

# ==== Executables ====
add_executable(simple_viz src/simple_viz.cpp)
target_link_libraries(simple_viz ouster_client ouster_viz)
add_executable(OusterSDK::simple_viz ALIAS simple_viz)

# ==== Install ====
install(TARGETS ouster_viz simple_viz
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
