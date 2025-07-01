find_package(Threads REQUIRED)

if(TARGET OusterSDK::ouster_pcap)
add_executable(client_example client_example.cpp)
target_link_libraries(client_example PRIVATE OusterSDK::ouster_sensor)

add_executable(client_packet_example client_packet_example.cpp)
target_link_libraries(client_packet_example PRIVATE OusterSDK::ouster_sensor)

add_executable(config_example config_example.cpp)
target_link_libraries(config_example PRIVATE OusterSDK::ouster_sensor)
else()
  message(STATUS "No ouster_sensor library available; skipping examples")
endif()

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
  
  add_executable(osf_writer_example osf_writer_example.cpp)
  target_link_libraries(osf_writer_example PRIVATE OusterSDK::ouster_osf)
else()
  message(STATUS "No ouster_osf library available; skipping examples")
endif()

if(TARGET OusterSDK::ouster_viz)
  add_executable(viz_example viz_example.cpp)
  target_link_libraries(viz_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)

  add_executable(viz_events_example viz_events_example.cpp)
  target_link_libraries(viz_events_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)

  add_executable(viz_screenshot_example viz_screenshot_example.cpp)
  target_link_libraries(viz_screenshot_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)
else()
  message(STATUS "No ouster_viz library available; skipping examples")
endif()

add_executable(scan_source_example scan_source_example.cpp)
target_link_libraries(scan_source_example PRIVATE OusterSDK::ouster_osf OusterSDK::ouster_sensor)
