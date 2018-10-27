cmake_minimum_required(VERSION 3.1.0)
project(ouster_viz)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS         "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(Threads)
find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED COMPONENTS vtkCommonCore NO_MODULE)

# Load additional vtk components; prefer newer vtk opengl module
# https://www.visitusers.org/index.php?title=VTK_7.1_Upgrade
# https://www.vtk.org/Wiki/VTK/Build_System_Migration
set(VIZ_VTK_COMPONENTS vtkCommonCore vtkCommonDataModel vtkCommonMath
  vtkCommonTransforms vtkFiltersSources vtkImagingCore vtkInteractionStyle
  vtkRenderingCore
  )
if(TARGET vtkRenderingOpenGL2)
  list(APPEND VIZ_VTK_COMPONENTS vtkRenderingOpenGL2)
else()
  list(APPEND VIZ_VTK_COMPONENTS vtkRenderingOpenGL)
endif()
find_package(VTK REQUIRED COMPONENTS ${VIZ_VTK_COMPONENTS} NO_MODULE)

include(${VTK_USE_FILE})

add_subdirectory(../ouster_client ouster_client EXCLUDE_FROM_ALL)

include_directories(include)

include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${VTK_INCLUDE_DIRS}
  )

add_executable(viz src/main.cpp src/viz.cpp)

target_link_libraries(viz
  ouster_client
  ${VTK_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
  )
