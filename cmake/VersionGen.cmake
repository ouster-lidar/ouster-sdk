cmake_minimum_required(VERSION 3.1.0)

set(GIT_HASH "unknown")

find_package(Git QUIET)

if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --match none --dirty
    OUTPUT_VARIABLE GIT_OUTPUT
    RESULT_VARIABLE GIT_RESULT
    ERROR_QUIET
    WORKING_DIRECTORY ${VERSION_GEN_SOURCE_DIR})

  if(${GIT_RESULT} EQUAL 0)
    set(GIT_HASH "${GIT_OUTPUT}")
  endif()
endif()

string(STRIP "${GIT_HASH}" GIT_HASH)
string(TOLOWER "${BUILD_TYPE}" BUILD_TYPE)

configure_file(${VERSION_GEN_SOURCE_DIR}/cmake/build.h.in ${VERSION_GEN_OUT_DIR}/build.h @ONLY)
