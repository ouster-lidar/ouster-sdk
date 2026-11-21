# Minimum macOS for Apple builds. Must be CACHE and included before project().
# https://stackoverflow.com/questions/34208360/cmake-seems-to-ignore-cmake-osx-deployment-target
# Keep OUSTER_OSX_DEPLOYMENT_TARGET in sync with build_libs.OUSTER_OSX_DEPLOYMENT_TARGET.
set(OUSTER_OSX_DEPLOYMENT_TARGET "14.0" CACHE STRING "Minimum macOS deployment target")
if(APPLE)
  set(CMAKE_OSX_DEPLOYMENT_TARGET "${OUSTER_OSX_DEPLOYMENT_TARGET}" CACHE STRING "" FORCE)
  message(STATUS "CMAKE_OSX_DEPLOYMENT_TARGET=${CMAKE_OSX_DEPLOYMENT_TARGET}")
endif()
