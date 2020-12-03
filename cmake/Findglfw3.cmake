# For ROS/catkin compatibility only. Set variables expected by catkin to allow exporting GLFW3 in
# the DEPENDS clause of the catkin_package() macro.

include(FindPackageHandleStandardArgs)

find_package(glfw3 CONFIG REQUIRED)

# Package on >=18.04 sets a target, 16.04 uses default vars
if (TARGET glfw)
  get_target_property(GLFW3_LIBRARIES glfw LOCATION)
  get_target_property(GLFW3_INCLUDE_DIRS glfw INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(GLFW3_INCLUDE_DIRS ${GLFW3_INCLUDE_DIR})
  set(GLFW3_LIBRARIES ${GLFW3_LIBRARY})
endif()

find_package_handle_standard_args(glfw3
  REQUIRED_VARS GLFW3_INCLUDE_DIRS GLFW3_LIBRARIES
  VERSION_VAR glfw3_VERSION
)
