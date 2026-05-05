# Allow downstream code to depend on source transparently
if(NOT TARGET NMEA_INCLUDED)
  add_custom_target(NMEA_INCLUDED)

  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/nmea
    ${CMAKE_CURRENT_BINARY_DIR}/nmea EXCLUDE_FROM_ALL)
endif()
