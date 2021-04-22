cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_pcap)

# ==== Requirements ====
find_package(libtins REQUIRED)

# ==== Libraries ====
add_library(ouster_pcap src/os_pcap.cpp)
target_link_libraries(ouster_pcap PUBLIC ouster_client tins)
target_include_directories(ouster_pcap PUBLIC include)

# some warnings coming from libtins
if(MSVC)
  target_compile_options(ouster_pcap PUBLIC /wd4251)
endif()
