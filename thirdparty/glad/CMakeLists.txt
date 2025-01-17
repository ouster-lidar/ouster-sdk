add_library(glad STATIC glad.c)
target_include_directories(glad PUBLIC 
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS glad
    EXPORT ouster-sdk-targets
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
)