# =======================================================================
# ======== Flatbuffers Generators for C++/Python/Javasctipt =============

# This file provides three generator functions that makes rules for convertion
# of Flatbuffer's specs (*.fbs) to the corresponding language stubs:
#  - 'build_cpp_fb_modules' - for C++ include headers
#  - 'build_ts_fb_modules' - for Typescript modules
#  - 'build_py_fb_modules' - for Python modules

# All functions have the common structure and use the common parameters
# (with binary schemas generators available only in C++):

#  TARGET  - (REQUIRED) Name of the target that will combine all custom
#            generator commands. It can later be used as dependency for other
#            targets (including ALL - to always generate code if needed)

#  FB_DIR   - (OPTIONAL) Root of Flatbuffers definitions, where specs organized
#              by modules. (see FB_MODULES param)
#              Example layout of FB_DIR ('project-luna/fb') for imaginary
#             'project-luna':
#                  - project-luna
#                    - fb
#                      - background_rad.fbs
#                      - rover_base
#                        - rover_state.fbs
#                        - sars_stream.fbs
#                      - laser_eye
#                        - photons_stream.fbs
#                        - neutrino_extrinsics.fbs
#             default: if ommited defaults to 'current-project/fb'
#                      (i.e. "${CMAKE_CURRENT_SOURCE_DIR}/fb")

#  FB_MODULES  - (OPTIONAL) List of modules (i.e. subdirectories of FB_DIR)
#                to use for code generation. The resulting subdirectories
#                structure is preserved (Typescript/C++ only).
#                For example (C++) file 'FB_DIR/rover_base/sars_stream.fbs' will
#                be converted to 'SOURCE_GEN_DIR/reover_base/sars_stream_generated.h'
#                For the 'project-luna' above we have two modules:
#                'rover_base' and 'laser_eye'.
#                default: empty, i.e. no modules used for generation and only
#                         root *.fbs files (FB_DIR/*.fbs) used. (except Python,
#                         where FB_MODULES is not used and all files recursively
#                         is generated)

#  SOURCE_GEN_DIR  - (OPTIONAL) Location of the generated source files.
#                    It's also available as TARGET property by SOURCE_GEN_DIR
#                    name which is usefull if using default SOURCE_GEN_DIR
#                    location.
#                    default: "${CMAKE_CURRENT_BINARY_DIR}/fb_source_generated"

#  BINARY_SCHEMA_DIR  - (OPTIONAL) Location of the generated binary schemas
#                        files (*.bfbs).
#                        It's also available as property BINARY_SCHEMA_DIR
#                        for generated TARGET.                     
#                     default: "${CMAKE_CURRENT_BINARY_DIR}/fb_binary_schemas"


# ======= Common Initializer =============================================

macro(initialize_osf_fb_utils_defaults)
  # Use flatc reolve from flatbuffers/CMake/BuildFlatBuffers.cmake source
  # Test if including from FindFlatBuffers
  if(FLATBUFFERS_FLATC_EXECUTABLE)
    set(FLATC ${FLATBUFFERS_FLATC_EXECUTABLE})
  else()
    set(FLATC flatc)
  endif()
  set(FB_DEFAULT_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/fb")
  set(FB_DEFAULT_SOURCE_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/fb_source_generated")
  set(FB_DEFAULT_BINARY_SCHEMA_DIR "${CMAKE_CURRENT_BINARY_DIR}/fb_binary_schemas")
endmacro()

# --- Log/debug helper function
# logs messages if FB_VERBOSE is true (in cmake sense, e.g. 1, Y, TRUE, YES, ON)
macro(fb_log MSG)
  if(FB_VERBOSE)
    message(STATUS ${MSG})
  endif()
endmacro()


# ======= C++ Generation =============================================

# --- Helper macro:
# Generates Flatbuffers binary schema generation target for FB_FILE in FB_MODULE
# Result placed in RESULT_DIR
# Side effect: Variable ALL_GEN_FILE appended with generated file.
macro(make_binary_schemas_command)
  set(PARSE_ONE_VALUE_ARGS FB_FILE FB_MODULE RESULT_DIR)
  cmake_parse_arguments(MOD_ARG "" "${PARSE_ONE_VALUE_ARGS}" "" ${ARGN})
  fb_log("BIN MOD: FB_FILE = ${MOD_ARG_FB_FILE}")
  fb_log("BIN MOD: FB_MODULE = ${MOD_ARG_FB_MODULE}")
  fb_log("BIN MOD: RESULT_DIR = ${MOD_ARG_RESULT_DIR}")
  if (NOT ${MOD_ARG_FB_MODULE} STREQUAL "")
    set(BIN_DIR ${MOD_ARG_RESULT_DIR}/${MOD_ARG_FB_MODULE})
  else()
    set(BIN_DIR ${MOD_ARG_RESULT_DIR})
  endif()
  get_filename_component(FB_NAME ${MOD_ARG_FB_FILE} NAME_WE)
  set(BIN_FILE ${BIN_DIR}/${FB_NAME}.bfbs)
  add_custom_command(
    OUTPUT ${BIN_FILE}
    DEPENDS ${MOD_ARG_FB_FILE}
    COMMAND
      ${FLATC} -b --schema
        -o ${BIN_DIR}
        ${MOD_ARG_FB_FILE}
    )
  list(APPEND ALL_GEN_FILES ${BIN_FILE})
endmacro()

# --- Helper macro:
# Generates Flatbuffers C++ schema generation target for FB_FILE in FB_MODULE
# Result placed in RESULT_DIR
# Side effect: Variable ALL_GEN_FILE appended with generated file.
macro(make_cpp_gen_command)
  set(PARSE_ONE_VALUE_ARGS FB_FILE FB_MODULE RESULT_DIR)
  cmake_parse_arguments(MOD_ARG "" "${PARSE_ONE_VALUE_ARGS}" "" ${ARGN})
  fb_log("CPP MOD: FB_FILE = ${MOD_ARG_FB_FILE}")
  fb_log("CPP MOD: FB_MODULE = ${MOD_ARG_FB_MODULE}")
  fb_log("CPP MOD: RESULT_DIR = ${MOD_ARG_RESULT_DIR}")
  if (NOT ${MOD_ARG_FB_MODULE} STREQUAL "")
    set(GEN_DIR ${MOD_ARG_RESULT_DIR}/${MOD_ARG_FB_MODULE})
  else()
    set(GEN_DIR ${MOD_ARG_RESULT_DIR})
  endif()
  get_filename_component(FB_NAME ${MOD_ARG_FB_FILE} NAME_WE)
  set(GEN_FILE ${GEN_DIR}/${FB_NAME}_generated.h)
  add_custom_command(
      OUTPUT ${GEN_FILE}
      DEPENDS ${MOD_ARG_FB_FILE}
      COMMAND
        ${FLATC} --cpp --no-includes --scoped-enums
          -o ${GEN_DIR}
          ${MOD_ARG_FB_FILE}
      )
    list(APPEND ALL_GEN_FILES ${GEN_FILE})
endmacro()

# C++ Flatbuffer generators
# Params: TARGET (REQUIRED), FB_DIR, FB_MODULES, SOURCE_GEN_DIR, BINARY_SCHEMA_DIR
# Properties available on generated TARGET: SOURCE_GEN_DIR, BINARY_SCHEMA_DIR
function(build_cpp_fb_modules)

  initialize_osf_fb_utils_defaults()

  set(PARSE_OPTIONS "")
  set(PARSE_ONE_VALUE_ARGS
        TARGET
        FB_DIR
        SOURCE_GEN_DIR
        BINARY_SCHEMA_DIR
        )
  set(PARSE_MULTI_VALUE_ARGS FB_MODULES)
  cmake_parse_arguments(ARGS "${PARSE_OPTIONS}" "${PARSE_ONE_VALUE_ARGS}"
    "${PARSE_MULTI_VALUE_ARGS}" ${ARGN} )

  fb_log("... BUILDING CPP FB MODULES ............ ")
  fb_log("ARGS_TARGET = ${ARGS_TARGET}")
  fb_log("ARGS_FB_DIR = ${ARGS_FB_DIR}")
  fb_log("ARGS_FB_MODULES = ${ARGS_FB_MODULES}")
  fb_log("ARG_SOURCE_GEN_DIR = ${ARGS_SOURCE_GEN_DIR}")
  fb_log("ARG_BINARY_GEN_DIR = ${ARGS_BINARY_SCHEMA_DIR}")

  if (NOT DEFINED ARGS_FB_DIR)
    set(ARGS_FB_DIR "${FB_DEFAULT_ROOT_DIR}")
    fb_log("using default FB_DIR = ${ARGS_FB_DIR}")
  endif()

  if (NOT DEFINED ARGS_SOURCE_GEN_DIR)
    set(ARGS_SOURCE_GEN_DIR "${FB_DEFAULT_SOURCE_GEN_DIR}/cpp")
    fb_log("using default SOURCE_GEN_DIR = ${ARGS_SOURCE_GEN_DIR}")
  endif()

  if (NOT DEFINED ARGS_BINARY_SCHEMA_DIR)
    set(ARGS_BINARY_SCHEMA_DIR "${FB_DEFAULT_BINARY_SCHEMA_DIR}")
    fb_log("using default BINARY_SCHEMA_DIR = ${ARGS_BINARY_SCHEMA_DIR}")
  endif()

  # List accumulated in macros 'make_*_command' calls
  set(ALL_GEN_FILES "")

  file(GLOB MOD_FB_CORE_FILES LIST_DIRECTORIES false ${ARGS_FB_DIR}/*.fbs)
  foreach(FB_FILE ${MOD_FB_CORE_FILES})

    make_cpp_gen_command(
      FB_FILE ${FB_FILE}
      RESULT_DIR ${ARGS_SOURCE_GEN_DIR}
      )

    if (NOT ${ARGS_BINARY_SCHEMA_DIR} STREQUAL "")
      make_binary_schemas_command(
        FB_FILE ${FB_FILE}
        RESULT_DIR ${ARGS_BINARY_SCHEMA_DIR}
        )
    endif()

  endforeach()

  foreach(FB_MODULE ${ARGS_FB_MODULES})
    file(GLOB FB_MODULE_FILES LIST_DIRECTORIES false ${ARGS_FB_DIR}/${FB_MODULE}/*.fbs)

    foreach(FB_FILE ${FB_MODULE_FILES})

      make_cpp_gen_command(
        FB_FILE ${FB_FILE}
        FB_MODULE ${FB_MODULE}
        RESULT_DIR ${ARGS_SOURCE_GEN_DIR}
        )

      if (NOT ${ARGS_BINARY_SCHEMA_DIR} STREQUAL "")
        make_binary_schemas_command(
          FB_FILE ${FB_FILE}
          FB_MODULE ${FB_MODULE}
          RESULT_DIR ${ARGS_BINARY_SCHEMA_DIR}
          )
      endif()

    endforeach()

  endforeach()

  add_custom_target(${ARGS_TARGET} DEPENDS ${ALL_GEN_FILES}
    COMMENT "Generating C++ code from Flatbuffers spec")

  set_property(TARGET ${ARGS_TARGET}
      PROPERTY SOURCE_GEN_DIR
      ${ARGS_SOURCE_GEN_DIR}
      )

  set_property(TARGET ${ARGS_TARGET}
      PROPERTY BINARY_SCHEMA_DIR
      ${ARGS_BINARY_SCHEMA_DIR}
      )

endfunction()


# ======= Typescript Generation ========================================

# --- Helper macro:
# Generates Flatbuffers Typescript generation command for FB_FILE in FB_MODULE
# Result placed in RESULT_DIR
# Side effect: Variable ALL_GEN_FILE appended with generated file.
macro(make_ts_gen_command)
  set(PARSE_ONE_VALUE_ARGS FB_FILE FB_MODULE RESULT_DIR)
  cmake_parse_arguments(MOD_ARG "" "${PARSE_ONE_VALUE_ARGS}" "" ${ARGN})
  fb_log("TS MOD: FB_FILE = ${MOD_ARG_FB_FILE}")
  fb_log("TS MOD: FB_MODULE = ${MOD_ARG_FB_MODULE}")
  fb_log("TS MOD: RESULT_DIR = ${MOD_ARG_RESULT_DIR}")
  if (NOT ${MOD_ARG_FB_MODULE} STREQUAL "")
    set(GEN_DIR ${MOD_ARG_RESULT_DIR}/${MOD_ARG_FB_MODULE})
  else()
    set(GEN_DIR ${MOD_ARG_RESULT_DIR})
  endif()
  get_filename_component(FB_NAME ${MOD_ARG_FB_FILE} NAME_WE)
  set(GEN_FILE ${GEN_DIR}/${FB_NAME}_generated.ts)
  add_custom_command(
    OUTPUT ${GEN_FILE}
    DEPENDS ${MOD_ARG_FB_FILE}
    COMMAND
      ${FLATC} --ts
        -o ${GEN_DIR}
        ${MOD_ARG_FB_FILE}
    )
  list(APPEND ALL_GEN_FILES ${GEN_FILE})
endmacro()

# Typescript Flatbuffer generators
# Params: TARGET (REQUIRED), FB_DIR, FB_MODULES, SOURCE_GEN_DIR
# Properties available on generated TARGET: SOURCE_GEN_DIR
function(build_ts_fb_modules)

  initialize_osf_fb_utils_defaults()

  set(PARSE_OPTIONS "")
  set(PARSE_ONE_VALUE_ARGS
        TARGET
        FB_DIR
        SOURCE_GEN_DIR
        )
  set(PARSE_MULTI_VALUE_ARGS FB_MODULES)
  cmake_parse_arguments(ARGS "${PARSE_OPTIONS}" "${PARSE_ONE_VALUE_ARGS}"
    "${PARSE_MULTI_VALUE_ARGS}" ${ARGN} )

  fb_log("...  BUILDING TS FB MODULES ............ ")
  fb_log("ARGS_TARGET = ${ARGS_TARGET}")
  fb_log("ARGS_FB_DIR = ${ARGS_FB_DIR}")
  fb_log("ARGS_FB_MODULES = ${ARGS_FB_MODULES}")
  fb_log("ARG_SOURCE_GEN_DIR = ${ARGS_SOURCE_GEN_DIR}")

  if (NOT DEFINED ARGS_FB_DIR)
    set(ARGS_FB_DIR "${FB_DEFAULT_ROOT_DIR}")
    fb_log("using default FB_DIR = ${ARGS_FB_DIR}")
  endif()

  if (NOT DEFINED ARGS_SOURCE_GEN_DIR)
    set(ARGS_SOURCE_GEN_DIR "${FB_DEFAULT_SOURCE_GEN_DIR}/ts")
    fb_log("using default SOURCE_GEN_DIR = ${ARGS_SOURCE_GEN_DIR}")
  endif()

  # List accumulated in macros 'make_*_command' calls
  set(ALL_GEN_FILES "")

  file(GLOB MOD_FB_CORE_FILES LIST_DIRECTORIES false ${ARGS_FB_DIR}/*.fbs)
  foreach(FB_FILE ${MOD_FB_CORE_FILES})

    make_ts_gen_command(
      FB_FILE ${FB_FILE}
      FB_MODULE ${FB_MODULE}
      RESULT_DIR ${ARGS_SOURCE_GEN_DIR}
      )

  endforeach()

  foreach(FB_MODULE ${ARGS_FB_MODULES})

    file(GLOB FB_MODULE_FILES LIST_DIRECTORIES false ${ARGS_FB_DIR}/${FB_MODULE}/*.fbs)

    foreach(FB_FILE ${FB_MODULE_FILES})

      make_ts_gen_command(
        FB_FILE ${FB_FILE}
        FB_MODULE ${FB_MODULE}
        RESULT_DIR ${ARGS_SOURCE_GEN_DIR}
        )

    endforeach()

  endforeach()

  add_custom_target(${ARGS_TARGET} DEPENDS ${ALL_GEN_FILES}
    COMMENT "Generating Typescript code from Flatbuffers spec")

  set_property(TARGET ${ARGS_TARGET}
      PROPERTY SOURCE_GEN_DIR
      ${ARGS_SOURCE_GEN_DIR}
      )

endfunction()

# ======= Python Generation =============================================

# Python Flatbuffer generators
# Params: TARGET (REQUIRED), FB_DIR, SOURCE_GEN_DIR
# Properties available on generated TARGET: SOURCE_GEN_DIR
function(build_py_fb_modules)

  initialize_osf_fb_utils_defaults()

  set(PARSE_OPTIONS "")
  set(PARSE_ONE_VALUE_ARGS
        TARGET
        FB_DIR
        SOURCE_GEN_DIR
        )
  set(PARSE_MULTI_VALUE_ARGS "")
  cmake_parse_arguments(ARGS "${PARSE_OPTIONS}" "${PARSE_ONE_VALUE_ARGS}"
    "${PARSE_MULTI_VALUE_ARGS}" ${ARGN} )
  fb_log("... BUILDING PYTHON FB MODULES ............ ")
  fb_log("ARGS_TARGET = ${ARGS_TARGET}")
  fb_log("ARGS_FB_DIR = ${ARGS_FB_DIR}")
  fb_log("ARG_SOURCE_GEN_DIR = ${ARGS_SOURCE_GEN_DIR}")

  if (NOT DEFINED ARGS_FB_DIR)
    set(ARGS_FB_DIR "${FB_DEFAULT_ROOT_DIR}")
    fb_log("using default FB_DIR = ${ARGS_FB_DIR}")
  endif()

  if (NOT DEFINED ARGS_SOURCE_GEN_DIR)
    set(ARGS_SOURCE_GEN_DIR "${FB_DEFAULT_SOURCE_GEN_DIR}/cpp")
    fb_log("using default SOURCE_GEN_DIR = ${ARGS_SOURCE_GEN_DIR}")
  endif()

  file(GLOB_RECURSE ALL_FBS_FILES LIST_DIRECTORIES false ${ARGS_FB_DIR}/*.fbs)

  # Using random file names to allow multiproject use of this function
  string(RANDOM LENGTH 4 PY_GEN_RANDOM)
  set(PY_GEN_TIMESTAMP_FILE ${CMAKE_CURRENT_BINARY_DIR}/py_gen_${PY_GEN_RANDOM}.timestamp)
  add_custom_command(
    OUTPUT ${PY_GEN_TIMESTAMP_FILE}
    DEPENDS ${ALL_FBS_FILES}
    COMMAND ${FLATC} --python -o ${ARGS_SOURCE_GEN_DIR} ${ALL_FBS_FILES}
    COMMAND ${CMAKE_COMMAND} -E touch ${PY_GEN_TIMESTAMP_FILE}
    )

  add_custom_target(${ARGS_TARGET} DEPENDS ${PY_GEN_TIMESTAMP_FILE}
    COMMENT "Generating Python code from Flatbuffers spec")

  set_property(TARGET ${ARGS_TARGET}
    PROPERTY SOURCE_GEN_DIR
    ${ARGS_SOURCE_GEN_DIR}
    )

endfunction()
