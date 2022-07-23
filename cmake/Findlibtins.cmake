# Package configuration on supported platforms:
# debian: only pkgconfig provided
# vcpkg: config.cmake, `tins` target and LIBTINS_INCLUDE_DIRS, LIBTINS_LIBRARIES for compat
# brew: same as vcpkg, but LIBTINS_INCLUDE_DIRS is wrong
# conan: `libtins::libtins` target and libtins_INCLUDE_DIRS, libtins_LIBRARIES

# Try to find a cmake config compatible with vcpkg and fall back to defining
# targets using pkgconfig.
include(FindPackageHandleStandardArgs)

# Prefer config, if found
find_package(libtins CONFIG QUIET)
if(libtins_FOUND AND TARGET tins)

  # The config in vcpkg and homebrew doesn't set interface include directories
  # on the target (since it's usually installed and included relative to the
  # system include path). That assumption fails if brew is installed under /opt.
  # The config does, however, set LIBTINS_INCLUDE_DIRS but bizzarely, this
  # appears to be set to a path under /tmp on homebrew, so we override it here.
  unset(LIBTINS_INCLUDE_DIRS)
  find_path(LIBTINS_INCLUDE_DIRS
    NAMES tins/tins.h)

  set(libtins_INCLUDE_DIRS ${LIBTINS_INCLUDE_DIRS})
  set(libtins_LIBRARIES ${LIBTINS_LIBRARIES})

  find_package_handle_standard_args(libtins CONFIG_MODE)
  return()
endif()

# Fall back to find_library with hints from pkgconfig
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_TINS QUIET libtins)

  if(PC_TINS_FOUND)
    set(libtins_VERSION ${PC_TINS_VERSION})
  endif()
endif()

find_library(libtins_LIBRARIES NAMES libtins.so libtins.dylib
  PATHS ${PC_TINS_LIBDIR} ${PC_TINS_LIBRARY_DIRS})

find_path(libtins_INCLUDE_DIRS
  NAMES tins/tins.h
  PATHS ${PC_TINS_INCLUDE_DIRS})

find_package_handle_standard_args(libtins
  REQUIRED_VARS libtins_LIBRARIES libtins_INCLUDE_DIRS
  VERSION_VAR libtins_VERSION)
