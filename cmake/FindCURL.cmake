# Define forwards-compatible imported target for old platforms (Ubuntu 18.04)
# Note: curl from VCPKG appears to completely ignore curl find modules despite
# CMAKE_MODULE_PATH settings

include(FindPackageHandleStandardArgs)

# Prefer package config if it exists
find_package(CURL CONFIG QUIET)
if(CURL_FOUND)
  find_package_handle_standard_args(CURL CONFIG_MODE)
  return()
endif()

# Trying to find with pkg-config
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(pkg_curl QUIET libcurl)
  if(pkg_curl_FOUND)
    set(CURL_VERSION_STRING ${pkg_curl_VERSION})
  endif()
endif()

find_path(CURL_INCLUDE_DIRS
  NAMES curl/curl.h
  HINTS ${pkg_curl_INCLUDE_DIRS})
mark_as_advanced(CURL_INCLUDE_DIRS)

# Linux/macos only
find_library(CURL_LIBRARIES NAMES
  curl
  HINTS ${pkg_curl_LIBRARY_DIRS})
mark_as_advanced(CURL_LIBRARIES)

# Check that we have everything that we need
find_package_handle_standard_args(CURL
  REQUIRED_VARS CURL_INCLUDE_DIRS CURL_LIBRARIES
  VERSION_VAR CURL_VERSION_STRING)

if(NOT TARGET CURL::libcurl)
  add_library(CURL::libcurl UNKNOWN IMPORTED)
  set_target_properties(CURL::libcurl PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CURL_INCLUDE_DIRS}"
    IMPORTED_LINK_INTERFACE_LANGUAGES "C"
    IMPORTED_LOCATION "${CURL_LIBRARIES}")
endif()
