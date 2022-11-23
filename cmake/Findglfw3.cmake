# For ROS/catkin compatibility only. Set variables expected by catkin to allow exporting GLFW3 in
# the DEPENDS clause of the catkin_package() macro.

include(FindPackageHandleStandardArgs)

find_package(glfw3 CONFIG REQUIRED)

# Package on >=18.04 sets a target
if(TARGET glfw)
  get_target_property(GLFW3_LIBRARIES glfw LOCATION)
  get_target_property(GLFW3_INCLUDE_DIRS glfw INTERFACE_INCLUDE_DIRECTORIES)
endif()

find_package_handle_standard_args(glfw3 CONFIG_MODE)
