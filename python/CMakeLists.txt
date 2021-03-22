cmake_minimum_required(VERSION 3.1.0)

set(OUSTER_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/sdk" CACHE STRING "SDK source directory")
file(TO_CMAKE_PATH "${OUSTER_SDK_PATH}" OUSTER_SDK_PATH)
message(STATUS "Ouster SDK location: ${OUSTER_SDK_PATH}")
list(APPEND CMAKE_MODULE_PATH ${OUSTER_SDK_PATH}/cmake)

project(python-ouster-client)
set(CMAKE_CXX_STANDARD 14)

find_package(pybind11 2.2 REQUIRED)
find_package(Eigen3 REQUIRED)
option(BUILD_VIZ OFF)
find_package(ouster_example REQUIRED)

pybind11_add_module(_sensor src/cpp/sensor.cpp)
set_target_properties(_sensor PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(_sensor PRIVATE ouster_client)
target_include_directories(_sensor SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

pybind11_add_module(_pcap src/cpp/pcap.cpp)
set_target_properties(_pcap PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(_pcap PRIVATE ouster_pcap)
target_include_directories(_pcap SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
