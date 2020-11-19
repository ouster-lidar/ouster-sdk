cmake_minimum_required(VERSION 3.1.0)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(DefaultBuildType)

# ==== Project Name ====
project(ouster_viz)

# ==== Requirements ====
find_package(ouster_client REQUIRED HINTS ../)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(Threads)
find_package(GLEW REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)

if(MSVC)
  add_compile_options(/W3)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
else()
  add_compile_options(-Wall -Wextra -Werror)
endif()

# ==== Libraries ====
add_library(ouster_viz STATIC src/point_viz.cpp src/cloud.cpp src/camera.cpp src/image.cpp)
set_target_properties(ouster_viz PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ouster_viz
  PUBLIC Threads::Threads glfw GLEW::GLEW OpenGL::GL
  PRIVATE ouster_client)
target_include_directories(ouster_viz PUBLIC include)

# ==== Executables ====
add_executable(simple_viz src/main.cpp)
target_link_libraries(simple_viz ouster_client ouster_viz)
