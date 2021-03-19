cmake_minimum_required(VERSION 3.1.0)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(ouster_pcap)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)
option(BUILD_VIZ OFF)

# ==== Requirements ====
find_package(libtins REQUIRED)

# ==== Libraries ====
add_library(ouster_pcap src/os_pcap.cpp)
target_link_libraries(ouster_pcap PUBLIC ouster_client tins)
target_include_directories(ouster_pcap PUBLIC include)
if(WIN32)
    target_compile_definitions(ouster_pcap PRIVATE NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN)
    target_link_libraries(ouster_pcap PUBLIC ws2_32)
endif()
