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
add_library(ouster_viz src/point_viz.cpp src/cloud.cpp src/camera.cpp src/image.cpp)
target_link_libraries(ouster_viz
  PUBLIC Threads::Threads glfw GLEW::GLEW OpenGL::GL
  PRIVATE ouster_client)
target_include_directories(ouster_viz PUBLIC include)

# ==== Executables ====
add_executable(simple_viz src/main.cpp)
target_link_libraries(simple_viz ouster_client ouster_viz)
