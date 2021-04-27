# Allow downstream code to depend on source transparently
if(NOT TARGET EXAMPLE_INCLUDED)
  add_custom_target(EXAMPLE_INCLUDED)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. ouster_example EXCLUDE_FROM_ALL)
endif()
