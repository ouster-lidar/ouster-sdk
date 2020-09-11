# Guard against multiple includes
if(NOT TARGET OUSTER_OSF_INCLUDED)
  add_custom_target(OUSTER_OSF_INCLUDED)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/ouster_osf)
endif()
