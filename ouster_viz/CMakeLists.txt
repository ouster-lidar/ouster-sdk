# ==== Requirements ====
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
include(Coverage)

find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PNG REQUIRED)

# ==== Libraries ====
# use only MPL-licensed parts of eigen
add_definitions(-DEIGEN_MPL2_ONLY)

add_library(ouster_viz STATIC src/point_viz.cpp src/cloud.cpp src/camera.cpp src/image.cpp
  src/gltext.cpp src/misc.cpp src/glfw.cpp src/framebuffer.cpp src/screenshot_utils.cpp)
target_link_libraries(ouster_viz
  PRIVATE Eigen3::Eigen glfw ${GL_LOADER} OpenGL::GL ouster_client glad PNG::PNG)
set_property(TARGET ouster_viz PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_viz PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
CodeCoverageFunctionality(ouster_viz)

if(BUILD_SHARED_LIBRARY)
  target_compile_definitions(ouster_viz PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()

target_include_directories(ouster_viz 
  PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
add_library(OusterSDK::ouster_viz ALIAS ouster_viz)

# ==== Install ====
install(TARGETS ouster_viz
        EXPORT ouster-sdk-targets
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
