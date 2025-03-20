# Find flatbuffers and set flatc 
# Set OUSTER_SKIP_FIND_PACKAGE_STANDARD to FALSE on APPLE 
# When set to TRUE, there are: flatbuffers/flatbuffers.h not found errors 
if(APPLE)
  set(OUSTER_SKIP_FIND_PACKAGE_STANDARD FALSE)
else()
  set(OUSTER_SKIP_FIND_PACKAGE_STANDARD TRUE)
endif()

if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
  include(FindPackageHandleStandardArgs)
endif()

find_package(Flatbuffers CONFIG QUIET NO_CMAKE_FIND_ROOT_PATH)

# The package is named FlatBuffers on some platforms, try that next
if(NOT Flatbuffers_FOUND)
    find_package(FlatBuffers CONFIG REQUIRED QUIET NO_CMAKE_FIND_ROOT_PATH)
endif()

if(NOT TARGET flatbuffers::flatbuffers)
    add_library(flatbuffers::flatbuffers INTERFACE IMPORTED)
    set_target_properties(flatbuffers::flatbuffers PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${FLATBUFFERS_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${FLATBUFFERS_LIBRARY}" 
    )
    message(STATUS "FlatBuffers found: ${FLATBUFFERS_INCLUDE_DIR}")
endif()

# Flatbuffers flatc resolution and different search name 'flatbuffers` with Conan
# NOTE2[pb]: 200221007: We changed Conan cmake package to look to `flatbuffers`
#   because it started failing out of blue :idk:scream: will see.
if(NOT CONAN_EXPORTED)
  if(NOT DEFINED FLATBUFFERS_FLATC_EXECUTABLE)
      set(FLATBUFFERS_FLATC_EXECUTABLE flatbuffers::flatc)
  endif()
  message(STATUS "FLATBUFFERS_FLATC_EXECUTABLE found: ${FLATBUFFERS_FLATC_EXECUTABLE}" )
else()
  if(WIN32)
    set(FLATBUFFERS_FLATC_EXECUTABLE flatc.exe)
  else()
    set(FLATBUFFERS_FLATC_EXECUTABLE flatc)
  endif()
  message(STATUS "flatbuffers found: ${Flatbuffers_DIR}" )
endif()


if(NOT OUSTER_SKIP_FIND_PACKAGE_STANDARD)
    # Find flatc executable
    if(NOT DEFINED FLATBUFFERS_FLATC_EXECUTABLE)
      if(WIN32)
          find_program(FLATBUFFERS_FLATC_EXECUTABLE NAMES flatc.exe)
      else()
          find_program(FLATBUFFERS_FLATC_EXECUTABLE NAMES flatc)
      endif()
    endif()

    # Find the FlatBuffers include directory
    find_path(FLATBUFFERS_INCLUDE_DIR NAMES flatbuffers/flatbuffers.h)
    find_package_handle_standard_args(
      Flatbuffers
      DEFAULT_MSG FLATBUFFERS_FLATC_EXECUTABLE FLATBUFFERS_INCLUDE_DIR
    )
endif()
