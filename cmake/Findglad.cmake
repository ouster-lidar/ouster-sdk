# Allow downstream code to depend on source transparently
if(NOT TARGET GLAD_INCLUDED)
  add_custom_target(GLAD_INCLUDED)

  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/glad
    ${CMAKE_CURRENT_BINARY_DIR}/glad EXCLUDE_FROM_ALL)
endif()
