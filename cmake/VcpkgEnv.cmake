# Make it possible to configure builds against vcpkg using env variables
# - must include() this before project() in CMakeLists.txt
# https://vcpkg.readthedocs.io/en/latest/users/integration/

# set toolchain file if VCPKG_ROOT is defind
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  message(STATUS "Using CMAKE_TOOLCHAIN_FILE from env VCPKG_ROOT: $ENV{VCPKG_ROOT}")
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "The CMake toolchain file")
endif()

# set VCPKG_TARGET_TRIPLET from corresponding env variable
if(DEFINED ENV{VCPKG_TARGET_TRIPLET} AND NOT DEFINED VCPKG_TARGET_TRIPLET)
  message(STATUS "Using VCPKG_TARGET_TRIPLET from env: $ENV{VCPKG_TARGET_TRIPLET}")
  set(VCPKG_TARGET_TRIPLET "$ENV{VCPKG_TARGET_TRIPLET}"
    CACHE STRING "Triplet used for vcpkg build")
endif()
