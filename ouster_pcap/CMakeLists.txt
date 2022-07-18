cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_pcap)

# ==== Project Packages ====
find_package(Pcap REQUIRED)
find_package(pcapplusplus REQUIRED)

# ==== Compile Options ====
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==== Libraries ====
add_library(ouster_pcap src/pcap.cpp src/os_pcap.cpp)
target_include_directories(ouster_pcap SYSTEM PRIVATE
  ${PCAP_INCLUDE_DIR})
target_include_directories(ouster_pcap PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(WIN32)
  target_compile_options(ouster_pcap PRIVATE /wd4200)
  target_link_libraries(ouster_pcap PUBLIC ws2_32)
endif()
target_link_libraries(ouster_pcap
  PRIVATE ${PCAP_LIBRARY} pcapplusplus::pcapplusplus)
add_library(OusterSDK::ouster_pcap ALIAS ouster_pcap)

# ==== Install ====
install(TARGETS ouster_pcap
  EXPORT ouster-sdk-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)

