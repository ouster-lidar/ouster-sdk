# ==== Requirements ====
find_package(libtins REQUIRED)

# ==== Libraries ====
add_library(ouster_pcap src/os_pcap.cpp)
target_include_directories(ouster_pcap SYSTEM PRIVATE
  ${PCAP_INCLUDE_DIR} ${libtins_INCLUDE_DIRS})
target_include_directories(ouster_pcap PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(WIN32)
  target_compile_options(ouster_pcap PRIVATE /wd4200)
  target_link_libraries(ouster_pcap PUBLIC ws2_32)
endif()
target_link_libraries(ouster_pcap PRIVATE libtins)
add_library(OusterSDK::ouster_pcap ALIAS ouster_pcap)

# ==== Install ====
install(TARGETS ouster_pcap
  EXPORT ouster-sdk-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
