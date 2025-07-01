# Prefer package config if it exists
find_package(tsl-robin-map CONFIG QUIET NO_CMAKE_FIND_ROOT_PATH)
if(tsl-robin-map_FOUND)
  set(robinMap_FOUND TRUE)
  return()
endif()

find_path(TSL_ROBIN_FOUND_PATH
  NAMES
    tsl/robin_set.h
    tsl/robin_map.h
    tsl/robin_hash.h
    tsl/robin_growth_policy.h)
if(TSL_ROBIN_FOUND_PATH)
  if(NOT TARGET tsl::robin_map)
    message(STATUS "Found tsl-robin-map: ${TSL_ROBIN_FOUND_PATH}")
    # Create imported target tsl::robin_map
    add_library(tsl::robin_map INTERFACE IMPORTED)

    set_target_properties(tsl::robin_map PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${TSL_ROBIN_FOUND_PATH}
    )
    get_target_property(tsl-robin-map_INCLUDE_DIRS tsl::robin_map INTERFACE_INCLUDE_DIRECTORIES)
    set(robinMap_FOUND TRUE)
    set(tsl-robin-map_FOUND TRUE)
    return()
  endif()
endif()
set(robinMap_FOUND FALSE)
