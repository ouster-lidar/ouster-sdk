# Targets defined for jsoncpp differ a good amount between distributions

include(FindPackageHandleStandardArgs)

find_package(jsoncpp CONFIG)

# This target exists on ubuntu / debian. For some reason debian bullseye doesn't
# ship a static lib / define the jsoncpp_lib_static target.
if(jsoncpp_FOUND AND TARGET jsoncpp_lib)
  find_package_handle_standard_args(jsoncpp DEFAULT_MSG jsoncpp_CONFIG)
# Target available in recent versions of vcpkg. We can make an alias of an
# imported target since we're guaranteed to be using a recent cmake
# Note: newer cmake will error if one but not both targets are in scope
elseif(jsoncpp_FOUND AND TARGET jsoncpp_static)
  set_target_properties(jsoncpp_static PROPERTIES IMPORTED_GLOBAL TRUE)
  set_target_properties(jsoncpp_object PROPERTIES IMPORTED_GLOBAL TRUE)
  add_library(jsoncpp_lib ALIAS jsoncpp_static)
  find_package_handle_standard_args(jsoncpp DEFAULT_MSG jsoncpp_CONFIG)
else()
  # fall back to pkg-config
  find_package(PkgConfig QUIET REQUIRED)
  pkg_check_modules(PC_JSONCPP QUIET jsoncpp)

  set(jsoncpp_VERSION ${PC_JSONCPP_VERSION})

  find_library(jsoncpp_LIBRARY NAMES libjsoncpp.so libjsoncpp.dylib
    PATHS ${PC_JSONCPP_LIBDIR} ${PC_JSONCPP_LIBRARY_DIRS})
  find_library(jsoncpp_STATIC_LIBRARY NAMES libjsoncpp.a
    PATHS ${PC_JSONCPP_LIBDIR} ${PC_JSONCPP_LIBRARY_DIRS})

  find_path(jsoncpp_INCLUDE_DIR
    NAMES json/json.h
    PATHS ${PC_JSONCPP_INCLUDE_DIRS}
    )

  find_package_handle_standard_args(jsoncpp
    REQUIRED_VARS jsoncpp_LIBRARY jsoncpp_INCLUDE_DIR
    VERSION_VAR json_VERSION
    )

  add_library(jsoncpp_lib_static STATIC IMPORTED)
  set_target_properties(jsoncpp_lib_static PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${jsoncpp_INCLUDE_DIR}
    IMPORTED_LOCATION ${jsoncpp_STATIC_LIBRARY})

  add_library(jsoncpp_lib SHARED IMPORTED)
  set_target_properties(jsoncpp_lib PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${jsoncpp_INCLUDE_DIR}
    IMPORTED_LOCATION ${jsoncpp_LIBRARY})
endif()
