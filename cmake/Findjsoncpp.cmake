# Targets defined for jsoncpp differ a good amount between distributions
include(FindPackageHandleStandardArgs)

# Recent jsoncpp cmake config fails if called twice
if(NOT jsoncpp_FOUND)
  find_package(jsoncpp CONFIG QUIET)
endif()

# This target exists on ubuntu / debian. For some reason debian bullseye doesn't
# ship a static lib / define the jsoncpp_lib_static target.
if(jsoncpp_FOUND AND TARGET jsoncpp_lib)
  find_package_handle_standard_args(jsoncpp CONFIG_MODE)
  return()
endif()

# With vcpkg, only a static lib is available so create a target for compatibility
if(jsoncpp_FOUND AND TARGET jsoncpp_static)
  add_library(jsoncpp_lib INTERFACE)
  target_link_libraries(jsoncpp_lib INTERFACE jsoncpp_static)
  install(TARGETS jsoncpp_lib EXPORT ouster-sdk-targets)
  find_package_handle_standard_args(jsoncpp CONFIG_MODE)
  return()
endif()

# Fall back to find_library with hints from pkgconfig
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_JSONCPP QUIET jsoncpp)
  if(PC_JSONCPP_FOUND)
    set(jsoncpp_VERSION_STRING ${PC_JSONCPP_VERSION})
  endif()
endif()

find_library(jsoncpp_LIBRARY NAMES libjsoncpp.so libjsoncpp.dylib
  PATHS ${PC_JSONCPP_LIBDIR} ${PC_JSONCPP_LIBRARY_DIRS})
find_library(jsoncpp_STATIC_LIBRARY NAMES libjsoncpp.a
  PATHS ${PC_JSONCPP_LIBDIR} ${PC_JSONCPP_LIBRARY_DIRS})

find_path(jsoncpp_INCLUDE_DIR
  NAMES json/json.h
  PATHS ${PC_JSONCPP_INCLUDE_DIRS})

find_package_handle_standard_args(jsoncpp
  REQUIRED_VARS jsoncpp_LIBRARY jsoncpp_INCLUDE_DIR
  VERSION_VAR jsoncpp_VERSION_STRING)

if(jsoncpp_FOUND)
  add_library(jsoncpp_lib_static STATIC IMPORTED)
  set_target_properties(jsoncpp_lib_static PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${jsoncpp_INCLUDE_DIR}
    IMPORTED_LOCATION ${jsoncpp_STATIC_LIBRARY})

  add_library(jsoncpp_lib SHARED IMPORTED)
  set_target_properties(jsoncpp_lib PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${jsoncpp_INCLUDE_DIR}
    IMPORTED_LOCATION ${jsoncpp_LIBRARY})
endif()
