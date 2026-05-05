cmake_minimum_required(VERSION 3.16)

if(NOT TARGET client_example OR NOT TARGET client_packet_example OR NOT TARGET config_example)
  message(STATUS "HIL example executables not available; skipping HIL tests")
  return()
endif()

set(SENSOR_HOSTNAME $ENV{SENSOR_HOSTNAME})
if(NOT SENSOR_HOSTNAME)
  message(WARNING "SENSOR_HOSTNAME env var not set; skipping tests")
  return()
endif()

message(STATUS "SENSOR_HOSTNAME: ${SENSOR_HOSTNAME}")

# Add HIL tests that run against a live sensor
add_test(NAME client_example
  COMMAND $<TARGET_FILE:client_example> ${SENSOR_HOSTNAME})

add_test(NAME client_packet_example
  COMMAND $<TARGET_FILE:client_packet_example> ${SENSOR_HOSTNAME})

add_test(NAME config_example
  COMMAND $<TARGET_FILE:config_example> ${SENSOR_HOSTNAME})

