cmake_minimum_required(VERSION 3.21)
project(pcap_test)

find_package(OusterSDK REQUIRED COMPONENTS Shared)

add_executable(pcap_test main.cpp)

target_link_libraries(pcap_test
    OusterSDK::shared_library
)
