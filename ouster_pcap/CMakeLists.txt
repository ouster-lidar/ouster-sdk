# ==== Requirements ====
find_package(Pcap REQUIRED)
find_package(libtins REQUIRED)

include(Coverage)

# ==== Libraries ====
add_library(ouster_pcap STATIC src/pcap.cpp src/os_pcap.cpp src/indexed_pcap_reader.cpp src/ip_reassembler.cpp src/pcap_packet_source.cpp src/pcap_scan_source.cpp)
target_include_directories(ouster_pcap SYSTEM PRIVATE
  ${PCAP_INCLUDE_DIR})
target_include_directories(ouster_pcap PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_property(TARGET ouster_pcap PROPERTY POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_pcap PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
CodeCoverageFunctionality(ouster_pcap)

if(BUILD_SHARED_LIBRARY)
  target_compile_definitions(ouster_pcap PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()

if(WIN32)
  target_link_libraries(ouster_pcap PUBLIC ws2_32)
endif()
target_link_libraries(ouster_pcap
  PUBLIC
    OusterSDK::ouster_client
    $<INSTALL_INTERFACE:libpcap::libpcap>
    $<INSTALL_INTERFACE:libtins::libtins>
  PRIVATE
    $<BUILD_INTERFACE:libpcap::libpcap>
    $<BUILD_INTERFACE:libtins::libtins>)
add_library(OusterSDK::ouster_pcap ALIAS ouster_pcap)

# ==== Install ====
install(TARGETS ouster_pcap
  EXPORT ouster-sdk-targets
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
