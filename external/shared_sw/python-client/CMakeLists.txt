cmake_minimum_required(VERSION 3.1.0)

# allows building with vcpkg if desired, by providing toolchain file path
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
  set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

project(python-ouster-client)

find_package(pybind11 2.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ouster_client REQUIRED)

pybind11_add_module(_sensor src/cpp/sensor.cpp)
set_target_properties(_sensor PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
target_link_libraries(_sensor PRIVATE ouster_client)
target_include_directories(_sensor SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
