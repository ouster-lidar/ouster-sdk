add_executable(client_example client_example.cpp)
target_link_libraries(client_example PRIVATE OusterSDK::ouster_client)

add_executable(mtp_client_example mtp_client_example.cpp)
target_link_libraries(mtp_client_example PRIVATE OusterSDK::ouster_client)

add_executable(config_example config_example.cpp)
target_link_libraries(config_example PRIVATE OusterSDK::ouster_client)

if(TARGET OusterSDK::ouster_pcap)
  add_executable(lidar_scan_example lidar_scan_example.cpp helpers.cpp)
  target_link_libraries(lidar_scan_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap)

  add_executable(representations_example representations_example.cpp helpers.cpp)
  target_link_libraries(representations_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap)
else()
  message(STATUS "No ouster_pcap library available; skipping examples")
endif()

if(TARGET OusterSDK::ouster_osf)
  add_executable(osf_reader_example osf_reader_example.cpp)
  target_link_libraries(osf_reader_example PRIVATE OusterSDK::ouster_osf)
else()
  message(STATUS "No ouster_osf library available; skipping examples")
endif()

if(TARGET OusterSDK::ouster_viz)
  add_executable(viz_example viz_example.cpp)
  target_link_libraries(viz_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)
else()
  message(STATUS "No ouster_viz library available; skipping examples")
endif()
