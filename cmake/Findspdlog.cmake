# Allow downstream code to depend on source transparently
if(NOT SPDLOG_INCLUDED)
  add_custom_target(SPDLOG_INCLUDED)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/spdlog
    ${CMAKE_CURRENT_BINARY_DIR}/spdlog EXCLUDE_FROM_ALL)
endif()
