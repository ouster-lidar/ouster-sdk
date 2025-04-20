# Try to find a cmake config compatible with vcpkg and fall back to defining
# targets using pkgconfig.
if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  include(FindPackageHandleStandardArgs)
endif()
# Prefer config, if found
find_package(zstd CONFIG QUIET)
if(NOT zstd_FOUND)
  # Fall back to find_library with hints from pkgconfig
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_ZSTD QUIET zstd)
  endif()
else()
add_library(zstd ALIAS zstd::libzstd)
endif()
