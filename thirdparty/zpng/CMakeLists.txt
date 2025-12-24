find_package(zstd REQUIRED)

add_library(zpng STATIC zpng.cpp)
target_include_directories(zpng PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(zpng
  PRIVATE
    zstd
)
install(TARGETS zpng
    EXPORT ouster-sdk-targets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)
