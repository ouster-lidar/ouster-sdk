add_executable(framebatcher_stats
    framebatcher_stats.cpp
)

target_link_libraries(framebatcher_stats
    Eigen3::Eigen
    ${LIB_TO_USE}
)
