if(NOT TARGET OUSTER_PCAP_INCLUDED)
  add_custom_target(OUSTER_PCAP_INCLUDED)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ ${CMAKE_CURRENT_BINARY_DIR}/ouster_pcap)
endif()
