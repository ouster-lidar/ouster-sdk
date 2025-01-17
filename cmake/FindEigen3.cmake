# Define forwards-compatible imported target for old platforms (Ubuntu 16.04)

if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  include(FindPackageHandleStandardArgs)
endif()

find_package(Eigen3 CONFIG QUIET NO_CMAKE_FIND_ROOT_PATH)

if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  find_package_handle_standard_args(Eigen3 CONFIG_MODE)
endif()

if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)

  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
endif()
