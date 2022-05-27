cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_pcap)

# ==== Requirements ====
find_package(libtins REQUIRED)
set(TINS_LIB tins)
if(CONAN_EXPORTED)
  # Conan requirements sourced, so use them
  find_package(libpcap REQUIRED)
  set(TINS_LIB libtins::libtins)
endif()

# ==== Libraries ====
add_library(ouster_pcap src/os_pcap.cpp)
target_link_libraries(ouster_pcap PRIVATE ${TINS_LIB})
# https://github.com/microsoft/vcpkg/issues/798
find_path(TINS_INCLUDE_DIR tins/tins.h)
target_include_directories(ouster_pcap PRIVATE
  ${TINS_INCLUDE_DIR}
  ${libpcap_INCLUDE_DIR}
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
add_library(OusterSDK::ouster_pcap ALIAS ouster_pcap)

# some warnings coming from libtins
if(MSVC)
  target_compile_options(ouster_pcap PUBLIC /wd4251)
endif()

# ==== Install ====
install(TARGETS ouster_pcap
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
