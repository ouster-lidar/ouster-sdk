# Allow downstream code to depend on source transparently
set(_ouster_robin_map_source_dir
    "${CMAKE_CURRENT_LIST_DIR}/../thirdparty/robin-map")

if(NOT EXISTS "${_ouster_robin_map_source_dir}/CMakeLists.txt")
  message(FATAL_ERROR
          "Vendored robin-map was not found at ${_ouster_robin_map_source_dir}")
endif()

if(NOT TARGET ROBIN_MAP_INCLUDED)
  add_custom_target(ROBIN_MAP_INCLUDED)
  add_subdirectory(${_ouster_robin_map_source_dir}
                   ${CMAKE_CURRENT_BINARY_DIR}/robin-map
                   EXCLUDE_FROM_ALL)
endif()

if(NOT TARGET tsl::robin_map)
  message(FATAL_ERROR
          "Vendored robin-map did not define expected target tsl::robin_map")
endif()

set(RobinMap_FOUND TRUE)
