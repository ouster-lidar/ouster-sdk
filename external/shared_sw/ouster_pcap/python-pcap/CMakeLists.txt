# allows building with vcpkg if desired, by providing toolchain file path
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
  set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

cmake_minimum_required(VERSION 3.1.0)

project(python-ouster-pcap)

find_package(pybind11 2.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ouster_client REQUIRED)
find_package(ouster_pcap REQUIRED)

pybind11_add_module(_pcap src/ouster_pyclient_pcap.cpp)
set_target_properties(_pcap PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(_pcap PRIVATE ouster_client ouster_pcap)
target_include_directories(_pcap SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
