cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_viz)

# ==== Requirements ====
set(OpenGL_GL_PREFERENCE GLVND)
find_package(Threads)
find_package(Eigen3 REQUIRED)
find_package(ouster_client REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall)

if(WIN32)
  find_package(glfw3 CONFIG REQUIRED)
  set(OUSTER_VIZ_ADDITIONAL_LIBS glfw GLEW::GLEW opengl32)
  add_definitions(-D_USE_MATH_DEFINES)
elseif(APPLE)
  find_package(glfw3 REQUIRED)
  set(OUSTER_VIZ_ADDITIONAL_LIBS glfw GLEW::GLEW OpenGL::GL)
  add_compile_options(-Wextra -Werror)
else()
  find_package(glfw3 REQUIRED)
  set(OUSTER_VIZ_ADDITIONAL_LIBS glfw GLEW GL)
  add_compile_options(-Wextra -Werror)
endif()

# ==== Libraries ====
add_library(ouster_viz STATIC src/point_viz.cpp src/cloud.cpp src/camera.cpp
                              src/image.cpp)
target_link_libraries(
  ouster_viz
  PUBLIC ${CMAKE_THREAD_LIBS_INIT}
  PRIVATE ouster_client ${OUSTER_VIZ_ADDITIONAL_LIBS})
target_include_directories(ouster_viz PUBLIC include)
target_include_directories(ouster_viz SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

# ==== Executables ====
add_executable(simple_viz src/main.cpp)
target_link_libraries(simple_viz ${CMAKE_THREAD_LIBS_INIT}
                      ${OUSTER_VIZ_ADDITIONAL_LIBS} ouster_client ouster_viz)
target_include_directories(simple_viz PRIVATE include)
target_include_directories(simple_viz SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
