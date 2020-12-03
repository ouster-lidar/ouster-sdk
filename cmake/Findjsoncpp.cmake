# Try to find a cmake config compatible with vcpkg and fall back to defining
# targets using pkgconfig, which seems more consistent for jsoncpp across
# different platforms.

include(FindPackageHandleStandardArgs)

# recent jsoncpp config shipped with vcpkg defines this target
find_package(jsoncpp CONFIG QUIET)
if(WIN32 OR (jsoncpp_FOUND AND TARGET jsoncpp_lib))
  find_package_handle_standard_args(jsoncpp DEFAULT_MSG jsoncpp_CONFIG)
  return()
endif()

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
