cmake_minimum_required(VERSION 3.1.0)

# === Libraries ===
add_library(ouster_example_helpers helpers.cpp)
target_link_libraries(ouster_example_helpers ouster_build ouster_client ouster_pcap)

# === Executables ===
add_executable(config_example config_example.cpp)
target_link_libraries(config_example PRIVATE ouster_client ouster_build ouster_example_helpers)
add_executable(OusterSDK::config_example ALIAS config_example)

add_executable(lidar_scan_example lidar_scan_example.cpp)
target_link_libraries(lidar_scan_example PRIVATE ouster_client ouster_build ouster_pcap ouster_example_helpers)
add_executable(OutserSDK::lidar_scan_example ALIAS lidar_scan_example)

add_executable(representations_example representations_example.cpp)
target_link_libraries(representations_example PRIVATE ouster_client ouster_build ouster_example_helpers)
add_executable(OusterSDK::ex ALIAS representations_example)
