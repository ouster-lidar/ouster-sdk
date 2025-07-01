# Allow downstream code to depend on source transparently
if(NOT TARGET KISS_ICP_INCLUDED)
  add_custom_target(KISS_ICP_INCLUDED)

  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/kiss-icp/cpp/kiss_icp
    ${CMAKE_CURRENT_BINARY_DIR}/kiss_icp EXCLUDE_FROM_ALL)
endif()
