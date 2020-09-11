# allows building with vcpkg if desired, by providing toolchain file path
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
  set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

cmake_minimum_required(VERSION 3.1.0)

project(ouster_pcap)

set(CMAKE_CXX_STANDARD 11)
SET(OUSTER_PCAP_ADDITIONAL_LIBS "")
SET(OUSTER_PCAP_ADDITIONAL_INCLUDES "")

IF(WIN32)
    SET(OUSTER_PCAP_ADDITIONAL_LIBS Version Iphlpapi ws2_32 Packet wpcap tins)
    SET(OUSTER_PCAP_ADDITIONAL_INCLUDES ${_VCPKG_INSTALLED_DIR}/x64-windows/include/)
    find_package(libtins REQUIRED)
else()
    set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(libtins REQUIRED libtins)
    SET(OUSTER_PCAP_ADDITIONAL_LIBS pcap ${libtins_LIBRARIES})
ENDIF()

include (CTest)
enable_testing()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Prepare packages

# Bring in OUSTER_CLIENT
get_filename_component(ABSOLUTE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../ouster_example" ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH ${ABSOLUTE_PATH})
find_package(ouster_client REQUIRED)
set_property(TARGET ouster_client PROPERTY POSITION_INDEPENDENT_CODE 1)

# Build OUSTER_PCAP lib
add_library(ouster_pcap STATIC
    src/os_pcap.cpp)

set_property(TARGET ouster_pcap PROPERTY POSITION_INDEPENDENT_CODE 1)
target_link_libraries(ouster_pcap PUBLIC ouster_client ${OUSTER_PCAP_ADDITIONAL_LIBS})
target_include_directories(ouster_pcap PUBLIC include ${OUSTER_PCAP_ADDITIONAL_INCLUDES} PRIVATE ${OUSTER_CLIENT_DIR}/include)
if(WIN32)
    target_link_directories(ouster_pcap PUBLIC ${_VCPKG_INSTALLED_DIR}/x64-windows/lib)
ENDIF()
