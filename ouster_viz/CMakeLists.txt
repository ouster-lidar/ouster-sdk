# ==== Requirements ====
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

# default to glad, if found. Note: this can be overridden from the command line
find_package(glad QUIET)
option(OUSTER_VIZ_USE_GLAD "Use GLAD instead of GLEW." ${glad_FOUND})
if(OUSTER_VIZ_USE_GLAD)
  message(STATUS "Configured GL loader: glad")
  find_package(glad REQUIRED)
  set(GL_LOADER glad::glad)
  add_definitions("-DOUSTER_VIZ_USE_GLAD")
else()
  message(STATUS "Configured GL loader: GLEW")
  find_package(GLEW REQUIRED)
  set(GL_LOADER GLEW::GLEW)
endif()

find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)

# ==== Libraries ====
# use only MPL-licensed parts of eigen
add_definitions(-DEIGEN_MPL2_ONLY)

add_library(ouster_viz src/point_viz.cpp src/cloud.cpp src/camera.cpp src/image.cpp
  src/gltext.cpp src/misc.cpp src/glfw.cpp)
target_link_libraries(ouster_viz
  PRIVATE Eigen3::Eigen glfw ${GL_LOADER} OpenGL::GL ouster_client)

target_include_directories(ouster_viz PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
add_library(OusterSDK::ouster_viz ALIAS ouster_viz)

# ==== Install ====
install(TARGETS ouster_viz
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
