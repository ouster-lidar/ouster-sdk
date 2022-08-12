# Package configuration on supported platforms:
# debian: only pkgconfig provided
# vcpkg: config.cmake, `tins` target and LIBTINS_INCLUDE_DIRS, LIBTINS_LIBRARIES for compat
# brew: same as vcpkg, but LIBTINS_INCLUDE_DIRS is wrong
# conan: `libtins`, `libtins::libtins` targets and libtins_INCLUDE_DIRS, libtins_LIBRARIES
#   libtins_LIBRARIES can't be used because it contains paths that differ between build and
#   install when building with conan

# Try to find a cmake config compatible with vcpkg and fall back to defining
# targets using pkgconfig.
include(FindPackageHandleStandardArgs)

# Prefer config, if found
find_package(libtins CONFIG QUIET)
if(libtins_FOUND AND TARGET tins)
  # The config in vcpkg and homebrew doesn't set interface include directories
  # on the target (since it's usually installed and included relative to the
  # system include path). That assumption fails if brew is installed under /opt.
  # The config does, however, set LIBTINS_INCLUDE_DIRS but bizarrely, this
  # appears to be set to a path under /tmp on homebrew, so we find the actual
  # include dir here and create our own target below.
  unset(LIBTINS_INCLUDE_DIRS)
  find_path(LIBTINS_INCLUDE_DIRS NAMES tins/tins.h)
  set_target_properties(tins PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES ${LIBTINS_INCLUDE_DIRS})

  find_package_handle_standard_args(libtins CONFIG_MODE)
else()
  # Fall back to find_library with hints from pkgconfig
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_TINS QUIET libtins)

    if(PC_TINS_FOUND)
      set(LIBTINS_VERSION ${PC_TINS_VERSION})
    endif()
  endif()

  find_library(LIBTINS_LIBRARIES NAMES libtins.so libtins.dylib
    PATHS ${PC_TINS_LIBDIR} ${PC_TINS_LIBRARY_DIRS})

  find_path(LIBTINS_INCLUDE_DIRS
    NAMES tins/tins.h
    PATHS ${PC_TINS_INCLUDE_DIRS})

  find_package_handle_standard_args(libtins
    REQUIRED_VARS LIBTINS_LIBRARIES LIBTINS_INCLUDE_DIRS
    VERSION_VAR LIBTINS_VERSION)
endif()

# LIBTINS_LIBRARIES is set, add target with a name compatible with the conan
# package
if(NOT TARGET libtins)
  add_library(libtins INTERFACE)
  set_target_properties(libtins PROPERTIES
    INTERFACE_LINK_LIBRARIES ${LIBTINS_LIBRARIES})
  install(TARGETS libtins EXPORT ouster-sdk-targets)
endif()
