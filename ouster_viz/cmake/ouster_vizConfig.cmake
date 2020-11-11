# Guard against multiple includes
if(NOT TARGET OUSTER_VIZ_INCLUDED)
  add_custom_target(OUSTER_VIZ_INCLUDED)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/ouster_viz)
endif()
