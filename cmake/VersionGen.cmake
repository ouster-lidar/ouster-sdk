if(CMAKE_SCRIPT_MODE_FILE)
  # in build stage
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

  configure_file(
    ${VERSION_GEN_SOURCE_DIR}/build.h.in
    ${VERSION_GEN_OUT_DIR}/build.h @ONLY)
elseif(NOT TARGET ouster_build)
  # in configuration stage: expects OusterSDK_VERSION_STRING to be set
  if(OusterSDK_VERSION_STRING MATCHES "^([0-9]+\.[0-9]+\.[0-9]+)(.([.0-9A-z]+))?$")
    set(OusterSDK_VERSION "${CMAKE_MATCH_1}")
    set(OusterSDK_VERSION_SUFFIX "${CMAKE_MATCH_3}")
  else()
    message(FATAL_ERROR "Failed to parse version string: ${OusterSDK_VERSION_STRING}")
  endif()

  add_custom_target(ouster_generate_header)
  add_custom_command(TARGET ouster_generate_header PRE_BUILD
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating build info header"
    COMMAND ${CMAKE_COMMAND}
    -DVERSION_GEN_OUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/generated/ouster/impl"
    -DVERSION_GEN_SOURCE_DIR="${CMAKE_CURRENT_LIST_DIR}"
    -DBUILD_TYPE="${CMAKE_BUILD_TYPE}"
    -DBUILD_SYSTEM="${CMAKE_SYSTEM_NAME}"
    -DOusterSDK_VERSION="${OusterSDK_VERSION}"
    -DOusterSDK_VERSION_SUFFIX="${OusterSDK_VERSION_SUFFIX}"
    -P ${CMAKE_CURRENT_LIST_FILE})

  add_library(ouster_build INTERFACE)
  target_include_directories(ouster_build INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>)
  add_dependencies(ouster_build ouster_generate_header)
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated/ouster
    DESTINATION include)

endif()
