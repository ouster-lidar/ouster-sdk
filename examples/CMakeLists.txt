# sensor_hostname needed for below tests so below are available with RUN_HIL_EXAMPLES and will run in HIL
# client_example, 
# client_packet_example
# config_example
# Viz examples skipped in CI and HIL

find_package(Threads REQUIRED)

option(RUN_CI_EXAMPLES "Add example executables to CTest" OFF)
option(RUN_HIL_EXAMPLES "Build HIL examples (client_example, client_packet_example, config_example)" OFF)

set(EXAMPLE_PCAP "${CMAKE_CURRENT_LIST_DIR}/../tests/pcaps/OS-1-128_767798045_1024x10_20230712_120049.pcap")
set(EXAMPLE_JSON "${CMAKE_CURRENT_LIST_DIR}/../tests/pcaps/OS-1-128_767798045_1024x10_20230712_120049.json")
set(EXAMPLE_PCAP_LARGE "$ENV{TEST_DATA_DIR}/bd-data/021521_os0_warehouse2_1024_trunc.pcap")
message(STATUS "EXAMPLE_PCAP_LARGE ${EXAMPLE_PCAP_LARGE}")
set(EXAMPLE_OSF "${CMAKE_CURRENT_LIST_DIR}/../tests/osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf")

if(TARGET OusterSDK::ouster_sensor)
add_executable(client_example client_example.cpp)
target_link_libraries(client_example PRIVATE OusterSDK::ouster_sensor)

add_executable(client_packet_example client_packet_example.cpp)
target_link_libraries(client_packet_example PRIVATE OusterSDK::ouster_sensor)

add_executable(config_example config_example.cpp)
target_link_libraries(config_example PRIVATE OusterSDK::ouster_sensor)

add_executable(zone_monitor_zone_set zone_monitor_zone_set.cpp)
target_link_libraries(zone_monitor_zone_set PRIVATE OusterSDK::ouster_client OusterSDK::ouster_sensor)

add_executable(zone_monitor_zone_states zone_monitor_zone_states.cpp)
target_link_libraries(zone_monitor_zone_states PRIVATE OusterSDK::ouster_client OusterSDK::ouster_sensor)

if(RUN_HIL_EXAMPLES)
  add_subdirectory(hil)
endif()
else()
  message(STATUS "No ouster_sensor library available; skipping examples")
endif()

if(TARGET OusterSDK::ouster_pcap)
  add_executable(lidar_scan_example lidar_scan_example.cpp helpers.cpp)
  target_link_libraries(lidar_scan_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap)
  if(RUN_CI_EXAMPLES)
    add_test(NAME lidar_scan_example COMMAND lidar_scan_example ${EXAMPLE_PCAP} ${EXAMPLE_JSON})
  endif()

  add_executable(representations_example representations_example.cpp helpers.cpp)
  target_link_libraries(representations_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_pcap)
  if(RUN_CI_EXAMPLES)
    add_test(NAME representations_example COMMAND representations_example ${EXAMPLE_PCAP} ${EXAMPLE_JSON})
  endif()
  else()
  message(STATUS "No ouster_pcap library available; skipping examples")
endif()

if(TARGET OusterSDK::ouster_osf)
  add_executable(osf_reader_example osf_reader_example.cpp)
  target_link_libraries(osf_reader_example PRIVATE OusterSDK::ouster_osf)
  if(RUN_CI_EXAMPLES)
    add_test(NAME osf_reader_example COMMAND osf_reader_example ${EXAMPLE_OSF})
  endif()
  
  add_executable(osf_writer_example osf_writer_example.cpp)
  target_link_libraries(osf_writer_example PRIVATE OusterSDK::ouster_osf)
  if(RUN_CI_EXAMPLES)
    add_test(NAME osf_writer_example COMMAND osf_writer_example test_output.osf)
  endif()
else()
  message(STATUS "No ouster_osf library available; skipping examples")
endif()

if(TARGET OusterSDK::ouster_viz)
  add_executable(viz_example viz_example.cpp)
  target_link_libraries(viz_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)

  add_executable(viz_mesh_example viz_mesh_example.cpp)
  target_link_libraries(viz_mesh_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)

  add_executable(viz_events_example viz_events_example.cpp)
  target_link_libraries(viz_events_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)

  add_executable(viz_screenshot_example viz_screenshot_example.cpp)
  target_link_libraries(viz_screenshot_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)

  add_executable(viz_notifications_example viz_notifications_example.cpp)
  target_link_libraries(viz_notifications_example PRIVATE OusterSDK::ouster_client OusterSDK::ouster_viz)
else()
  message(STATUS "No ouster_viz library available; skipping examples")
endif()

function(link_sdk_libraries_conditionally target_name)
    target_link_libraries(${target_name} PRIVATE OusterSDK::ouster_client)
    if (TARGET OusterSDK::ouster_osf)
      target_compile_definitions(${target_name} PUBLIC OUSTER_OSF)
      target_link_libraries(${target_name} PRIVATE OusterSDK::ouster_osf)
    endif()
    if (TARGET OusterSDK::ouster_pcap)
      target_compile_definitions(${target_name} PUBLIC OUSTER_PCAP)
      target_link_libraries(${target_name} PRIVATE OusterSDK::ouster_pcap)
    endif()
    if (TARGET OusterSDK::ouster_sensor)
      target_compile_definitions(${target_name} PUBLIC OUSTER_SENSOR)
      target_link_libraries(${target_name} PRIVATE OusterSDK::ouster_sensor)
    endif()
endfunction()

add_executable(scan_source_example scan_source_example.cpp)
link_sdk_libraries_conditionally(scan_source_example)
if(RUN_CI_EXAMPLES)
  add_test(NAME scan_source_example COMMAND scan_source_example ${EXAMPLE_PCAP_LARGE})
endif()

if(TARGET OusterSDK::ouster_mapping)
  add_executable(slam_example slam_example.cpp)
    link_sdk_libraries_conditionally(slam_example)
    target_link_libraries(slam_example PRIVATE OusterSDK::ouster_mapping)
    if(RUN_CI_EXAMPLES)
      add_test(NAME slam_example COMMAND slam_example ${EXAMPLE_PCAP})
    endif()
else()
  message(STATUS "No ouster_mapping library available; skipping SLAM example")
endif()
