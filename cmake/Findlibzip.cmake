if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  include(FindPackageHandleStandardArgs)
endif()

macro(libzip_internal_find_package)
  find_package(libzip CONFIG QUIET)
  if(libzip_FOUND)
    if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
      find_package_handle_standard_args(libzip CONFIG_MODE)
    endif()
    return()
  endif()
endmacro()


# If vcpkg prefer find_package
if(VCPKG_TOOLCHAIN)
  libzip_internal_find_package()
endif()

# Try to find with pkg-config first because the cmakelists is often bad for libzip
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(pkg_libzip QUIET libzip)
  if(pkg_libzip_FOUND)
    set(libzip_VERSION_STRING ${pkg_libzip_VERSION})
  endif()
endif()

# Otherwise fall back to find_package
if(NOT pkg_libzip_FOUND)
  libzip_internal_find_package()
endif()

find_path(libzip_INCLUDE_DIRS
  NAMES zip.h
  HINTS ${pkg_libzip_INCLUDE_DIRS})
mark_as_advanced(libzip_INCLUDE_DIRS)

# Linux/macos only
find_library(libzip_LIBRARIES NAMES
  libzip libzip.so libzip.dylib
  HINTS ${pkg_libzip_LIBRARY_DIRS})
mark_as_advanced(libzip_LIBRARIES)

# Check that we have everything that we need
if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  find_package_handle_standard_args(libzip
    REQUIRED_VARS libzip_INCLUDE_DIRS libzip_LIBRARIES
    VERSION_VAR libzip_VERSION_STRING)
endif()

if(NOT TARGET libzip::zip)
  add_library(libzip::zip UNKNOWN IMPORTED)
  set_target_properties(libzip::zip PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${libzip_INCLUDE_DIRS}"
    IMPORTED_LINK_INTERFACE_LANGUAGES "C"
    IMPORTED_LOCATION "${libzip_LIBRARIES}")
endif()
