# Allow downstream code to depend on source transparently
if(NOT TARGET SOPHUS_INCLUDED)
  add_custom_target(SOPHUS_INCLUDED)

  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/sophus 
    ${CMAKE_CURRENT_BINARY_DIR}/sophus EXCLUDE_FROM_ALL)
endif()
