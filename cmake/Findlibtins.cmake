# Try to find a cmake config compatible with vcpkg and fall back to defining
# targets using pkgconfig.

include(FindPackageHandleStandardArgs)

# recent libtins config shipped with vcpkg defines this target
find_package(libtins CONFIG QUIET)
if(WIN32 OR (libtins_FOUND AND TARGET tins))
  find_package_handle_standard_args(libtins DEFAULT_MSG libtins_CONFIG)
  return()
endif()

# fall back to pkg-config
find_package(PkgConfig QUIET REQUIRED)
pkg_check_modules(PC_TINS REQUIRED libtins)

set(libtins_VERSION ${PC_TINS_VERSION})

find_library(tins_LIBRARY NAMES libtins.so libtins.dylib
  PATHS ${PC_TINS_LIBDIR} ${PC_TINS_LIBRARY_DIRS})

find_path(tins_INCLUDE_DIR
    NAMES tins/tins.h
    PATHS ${PC_TINS_INCLUDE_DIRS}
)

find_package_handle_standard_args(libtins
    REQUIRED_VARS tins_LIBRARY tins_INCLUDE_DIR
    VERSION_VAR tins_VERSION
)

add_library(tins SHARED IMPORTED)
set_target_properties(tins PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${tins_INCLUDE_DIR}
  IMPORTED_LOCATION ${tins_LIBRARY})
