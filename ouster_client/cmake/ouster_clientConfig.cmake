if(NOT TARGET OUSTER_CLIENT_INCLUDED)
  add_custom_target(OUSTER_CLIENT_INCLUDED)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/ouster_client)
endif()
