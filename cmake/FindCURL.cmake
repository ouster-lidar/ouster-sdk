# Define forwards-compatible imported target for old platforms
# Note: curl from VCPKG appears to completely ignore curl find modules despite
# CMAKE_MODULE_PATH settings

if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  include(FindPackageHandleStandardArgs)
endif()

# Prefer package config if it exists
find_package(CURL CONFIG QUIET NO_CMAKE_FIND_ROOT_PATH)
if(CURL_FOUND)
  if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
    find_package_handle_standard_args(CURL CONFIG_MODE)
  endif()
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
if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  find_package_handle_standard_args(CURL
    REQUIRED_VARS CURL_INCLUDE_DIRS CURL_LIBRARIES
    VERSION_VAR CURL_VERSION_STRING)
endif()

if(NOT TARGET CURL::libcurl)
  add_library(CURL::libcurl UNKNOWN IMPORTED)
  set_target_properties(CURL::libcurl PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CURL_INCLUDE_DIRS}"
    IMPORTED_LINK_INTERFACE_LANGUAGES "C"
    IMPORTED_LOCATION "${CURL_LIBRARIES}")
endif()
