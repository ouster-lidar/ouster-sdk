cmake_minimum_required(VERSION 3.1.0)

# === Executables ===
add_executable(client_example client_example.cpp)
target_link_libraries(client_example PRIVATE ouster_client)

add_executable(config_example config_example.cpp helpers.cpp)
target_link_libraries(config_example PRIVATE ouster_client ouster_pcap)

add_executable(lidar_scan_example lidar_scan_example.cpp helpers.cpp)
target_link_libraries(lidar_scan_example PRIVATE ouster_client ouster_pcap)

add_executable(representations_example representations_example.cpp helpers.cpp)
target_link_libraries(representations_example PRIVATE ouster_client ouster_pcap)
