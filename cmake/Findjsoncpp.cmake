# Targets defined for jsoncpp differ a good amount between distributions

include(FindPackageHandleStandardArgs)

find_package(jsoncpp CONFIG REQUIRED QUIET)

# This target exists on ubuntu / debian. For some reason debian bullseye doesn't
# ship a static lib / define the jsoncpp_lib_static target.
if(TARGET jsoncpp_lib)
  find_package_handle_standard_args(jsoncpp DEFAULT_MSG jsoncpp_CONFIG)
# Target available in recent versions of vcpkg. We can make an alias of an
# imported target since we're guaranteed to be using a recent cmake
elseif(TARGET jsoncpp_static)
  set_target_properties(jsoncpp_static PROPERTIES IMPORTED_GLOBAL TRUE)
  add_library(jsoncpp_lib ALIAS jsoncpp_static)
endif()

find_package_handle_standard_args(jsoncpp DEFAULT_MSG jsoncpp_CONFIG)

