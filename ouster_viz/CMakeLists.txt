cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_viz)

# ==== Requirements ====
find_package(Threads)
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtkCommonCore NO_MODULE)
find_package(ouster_client REQUIRED)

# Load additional vtk components; prefer newer vtk opengl module
# https://www.visitusers.org/index.php?title=VTK_7.1_Upgrade
# https://www.vtk.org/Wiki/VTK/Build_System_Migration
set(VIZ_VTK_COMPONENTS vtkCommonCore vtkCommonDataModel vtkCommonMath
  vtkCommonTransforms vtkFiltersSources vtkImagingCore vtkInteractionStyle
  vtkRenderingCore vtkRenderingLabel
  )
if(TARGET vtkRenderingOpenGL2)
  list(APPEND VIZ_VTK_COMPONENTS vtkRenderingOpenGL2)
else()
  list(APPEND VIZ_VTK_COMPONENTS vtkRenderingOpenGL)
endif()
find_package(VTK REQUIRED COMPONENTS ${VIZ_VTK_COMPONENTS} NO_MODULE)

include(${VTK_USE_FILE})

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall)

if(NOT WIN32)
  add_compile_options(-Wextra -Werror)
endif()

# ==== Libraries ====
add_library(ouster_viz STATIC src/viz.cpp)
target_link_libraries(ouster_viz
  PUBLIC
  ${VTK_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  PRIVATE
  ouster_client
)
target_include_directories(ouster_viz PUBLIC include)
target_include_directories(ouster_viz SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR} ${VTK_INCLUDE_DIRS})

# ==== Executables ====
add_executable(simple_viz src/main.cpp)
target_link_libraries(simple_viz
  ${VTK_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  ouster_client
  ouster_viz
)
target_include_directories(simple_viz PRIVATE include)
target_include_directories(simple_viz SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

