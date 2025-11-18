cmake_minimum_required(VERSION 3.15)

project(ouster_c_wrapper LANGUAGES C CXX)

add_library(ouster_c SHARED
    src/ouster_c.cpp
    src/ouster_scan_source_c.cpp
)

target_compile_definitions(ouster_c PRIVATE OUSTER_C_BUILD)

target_include_directories(ouster_c
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

set_target_properties(ouster_c PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
    OUTPUT_NAME ouster_c
)

target_link_libraries(ouster_c
    PRIVATE
        ouster_sensor
        ouster_client
)

install(TARGETS ouster_c
    EXPORT ouster-sdk-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES include/ouster_c.h DESTINATION include)
