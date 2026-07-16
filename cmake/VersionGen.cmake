if(CMAKE_SCRIPT_MODE_FILE)
  # in build stage
  set(GIT_HASH "unknown")
  set(GIT_BRANCH "unknown")

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

  # Branch name: Jenkins BRANCH_NAME when set (prefixed with CI_), else local git.
  if(NOT "$ENV{BRANCH_NAME}" STREQUAL "")
    set(GIT_BRANCH "CI_$ENV{BRANCH_NAME}")
  elseif(GIT_FOUND)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
      OUTPUT_VARIABLE GIT_BRANCH_OUT
      RESULT_VARIABLE GIT_BRANCH_RESULT
      ERROR_QUIET
      WORKING_DIRECTORY ${VERSION_GEN_SOURCE_DIR})
    if(${GIT_BRANCH_RESULT} EQUAL 0)
      string(STRIP "${GIT_BRANCH_OUT}" GIT_BRANCH_OUT)
      if(GIT_BRANCH_OUT STREQUAL "HEAD")
        execute_process(
          COMMAND ${GIT_EXECUTABLE} name-rev --name-only HEAD
          OUTPUT_VARIABLE GIT_NAME_REV_OUT
          RESULT_VARIABLE GIT_NAME_REV_RESULT
          ERROR_QUIET
          WORKING_DIRECTORY ${VERSION_GEN_SOURCE_DIR})
        if(${GIT_NAME_REV_RESULT} EQUAL 0)
          string(STRIP "${GIT_NAME_REV_OUT}" GIT_NAME_REV_OUT)
          if(NOT GIT_NAME_REV_OUT STREQUAL "")
            set(GIT_BRANCH "${GIT_NAME_REV_OUT}")
          else()
            set(GIT_BRANCH "detached")
          endif()
        else()
          set(GIT_BRANCH "detached")
        endif()
      else()
        set(GIT_BRANCH "${GIT_BRANCH_OUT}")
      endif()
    endif()
  endif()

  configure_file(
    ${VERSION_GEN_SOURCE_DIR}/build.cpp.in
    ${VERSION_GEN_OUT_DIR}/build.cpp @ONLY)
elseif(NOT TARGET ouster_build)
  # in configuration stage: expects OusterSDK_VERSION_STRING to be set
  if(NOT OusterSDK_VERSION_STRING)
    message(FATAL_ERROR "OusterSDK_VERSION_STRING is not set")
  endif()

  add_custom_command(OUTPUT
      fake_file_to_rerun # uncomment me if you want this to run every build
      ${CMAKE_CURRENT_BINARY_DIR}/generated/ouster/impl/build.cpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating build info header"
    COMMAND ${CMAKE_COMMAND}
    -DVERSION_GEN_OUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/generated/ouster/impl"
    -DVERSION_GEN_SOURCE_DIR="${CMAKE_CURRENT_LIST_DIR}"
    -DBUILD_TYPE="${CMAKE_BUILD_TYPE}"
    -DBUILD_SYSTEM="${CMAKE_SYSTEM_NAME}"
    -DOusterSDK_VERSION_STRING="${OusterSDK_VERSION_STRING}"
    -P ${CMAKE_CURRENT_LIST_FILE}
    DEPENDS ${CMAKE_CURRENT_LIST_DIR}/build.cpp.in)

  set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/generated/ouster/impl/build.cpp
    PROPERTIES GENERATED TRUE)

  add_library(ouster_build STATIC ${CMAKE_CURRENT_BINARY_DIR}/generated/ouster/impl/build.cpp)
  set_target_properties(ouster_build PROPERTIES POSITION_INDEPENDENT_CODE ON)

  target_include_directories(ouster_build
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../ouster_core/include>
  )

endif()
