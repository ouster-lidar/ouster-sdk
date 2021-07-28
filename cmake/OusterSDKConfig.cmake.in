message(STATUS "Found OusterSDK: ${CMAKE_CURRENT_LIST_FILE}")

include(CMakeFindDependencyMacro)

# ouster_client dependencies
find_dependency(Eigen3)
find_dependency(jsoncpp)

# viz dependencies
set(OpenGL_GL_PREFERENCE GLVND)
find_dependency(OpenGL)
find_dependency(Threads)
find_dependency(GLEW)
find_dependency(glfw3)

# pcap dependencies (no cmake config on debian 10)
# find_dependency(libtins)

include("${CMAKE_CURRENT_LIST_DIR}/OusterSDKTargets.cmake")
