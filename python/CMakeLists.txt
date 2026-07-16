cmake_minimum_required(VERSION 3.16...3.22)

set(OUSTER_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/.." CACHE STRING "SDK source directory")
file(TO_CMAKE_PATH "${OUSTER_SDK_PATH}" OUSTER_SDK_PATH)
message(STATUS "Ouster SDK location: ${OUSTER_SDK_PATH}")
list(APPEND CMAKE_MODULE_PATH ${OUSTER_SDK_PATH}/cmake)

# configure vcpkg from environment variables, if present
include(VcpkgEnv)
include(Coverage)

set(VCPKG_MANIFEST_DIR ${OUSTER_SDK_PATH})
option(BUILD_PERCEPTION "Enabled for Python build" ON)
if(BUILD_PERCEPTION)
  # this has to be done before project() is called
  list(APPEND VCPKG_MANIFEST_FEATURES "perception")
endif()

project(python-ouster-sdk)

# ==== Options ====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
  # /MP – multi-processor compilation (parallel within a single CL.exe invocation)
  # /FS – force synchronous PDB writes required when /MP threads share a PDB file
  # /bigobj – allow more than 65535 COFF sections (needed for heavy template TUs)
  add_compile_options(/W2 /wd4996 /MP /FS /bigobj)
  add_compile_definitions(NOMINMAX _USE_MATH_DEFINES WIN32_LEAN_AND_MEAN _ENABLE_EXTENDED_ALIGNED_STORAGE)
else()
  add_compile_options(-Wall -Wextra -Wno-error=deprecated-declarations)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Parallelize linking optimization for GCC
    add_link_options(-flto=auto)
  endif()
  if (NOT APPLE AND NOT MSVC)
    # GCC/Clang accept -fuse-ld=lld (driver name), not -fuse-ld=/path/to/ld.lld.
    find_program(LLD_PROGRAM NAMES ld.lld lld)
    if (LLD_PROGRAM)
      add_link_options(-fuse-ld=lld)
    endif()
  endif()
endif()

option(BUILD_SENSOR "Enabled for Python build" ON)
option(BUILD_VIZ "Enabled for Python build" ON)
option(BUILD_OSF "Build OSF library." ON)
option(BUILD_PCAP "Enabled for Python build" ON)
option(BUILD_MAPPING "Enabled for Python build" ON)
option(SKIP_SDK_FIND "Skip finding the sdk" OFF)
option(BUILD_TESTING "" OFF)
option(BUILD_EXAMPLES "" OFF)

# ==== Requirements ====
find_package(Eigen3 REQUIRED)
if(NOT SKIP_SDK_FIND)
  if(DEFINED OUSTER_SDK_PREBUILT_DIR OR DEFINED ENV{OUSTER_SDK_PREBUILT_DIR})
    if(DEFINED ENV{OUSTER_SDK_PREBUILT_DIR} AND NOT DEFINED OUSTER_SDK_PREBUILT_DIR)
      set(OUSTER_SDK_PREBUILT_DIR "$ENV{OUSTER_SDK_PREBUILT_DIR}")
    endif()
    message(STATUS "Python build: using pre-built OusterSDK from ${OUSTER_SDK_PREBUILT_DIR}")
    find_package(OpenMP QUIET)
    find_package(OusterSDK CONFIG REQUIRED
      PATHS "${OUSTER_SDK_PREBUILT_DIR}/lib/cmake/OusterSDK"
      NO_DEFAULT_PATH)
  else()
    find_package(OusterSDK REQUIRED)
  endif()
endif()

# CMAKE_LIBRARY_OUTPUT_DIRECTORY is set in setup.py to the root of the `ouster`
# namespace, but we have to provide per-target packages directories for each
# extension module here.
set(EXT_DIR ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdk)

if (CMAKE_VERSION VERSION_LESS 3.18)
  set(DEV_MODULE Development)
else()
  set(DEV_MODULE Development.Module)
endif()

# Get the version of the python used to execute this build
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import platform; print(platform.python_version())"
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE PYTHON_VERSION
    RESULT_VARIABLE PYTHON_RESULT
    ERROR_VARIABLE PYTHON_ERROR)
if (NOT PYTHON_RESULT EQUAL "0")
    message(FATAL_ERROR "Failed to locate Python via 'PYTHON_EXECUTABLE' command: ${PYTHON_ERROR}")
endif()

# Set the Python executable for find_package
set(Python_EXECUTABLE ${PYTHON_EXECUTABLE})

# Find the EXACT required Python components
find_package(Python ${PYTHON_VERSION} COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)

# Locate nanobind via python
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT
  RESULT_VARIABLE nanobind_RESULT
  ERROR_VARIABLE nanobind_ERROR)

if (NOT nanobind_RESULT EQUAL "0")
    message(FATAL_ERROR "Failed to locate nanobind via Python: ${nanobind_ERROR}")
endif()

# Locate numpy include path via python
execute_process(
  COMMAND "${PYTHON_EXECUTABLE}" -c "import numpy; print(numpy.get_include())"
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE numpy_INCLUDE_PATH
  RESULT_VARIABLE numpy_RESULT
  ERROR_VARIABLE numpy_ERROR)
if (NOT numpy_RESULT EQUAL "0")
    message(FATAL_ERROR "Failed to locate numpy include path via Python: ${numpy_ERROR}")
endif()

find_package(nanobind CONFIG REQUIRED)

# Warning: All of the python binding code must be within the same static library
# otherwise you will get random segfaults due to how nanobind works.
#
# Note: With multi-configuration generators (like for VS), CMake automatically
# appends build-configuration suffix to *_OUTPUT_DIRECTORY properties *unless*
# they contain a generator expression, so we use a noop: $<0:>
# https://cmake.org/cmake/help/latest/prop_tgt/LIBRARY_OUTPUT_DIRECTORY.html
nanobind_add_module(_bindings
  src/cpp/main.cpp
  src/cpp/common.cpp
  src/cpp/_algorithm.cpp
  src/cpp/client/client_common.cpp
  src/cpp/client/client.cpp
  src/cpp/client/main.cpp
  src/cpp/client/misc.cpp
  src/cpp/client/data.cpp
  src/cpp/client/field.cpp
  src/cpp/client/lidar_frame.cpp
  src/cpp/client/object.cpp
  src/cpp/client/packet.cpp
  src/cpp/client/processing.cpp
  src/cpp/client/pose.cpp
  src/cpp/client/frame_ops.cpp
  src/cpp/client/frame_set_source.cpp
  src/cpp/client/zone_monitor.cpp
  src/cpp/_pcap.cpp
  src/cpp/_viz.cpp
  src/cpp/_osf.cpp
  src/cpp/mapping/_mapping.cpp
  src/cpp/mapping/_mapping_config.cpp
  src/cpp/mapping/_mapping_constraints.cpp
  src/cpp/mapping/_mapping_pose_optimizer.cpp
  src/cpp/mapping/_mapping_slam.cpp
  src/cpp/mapping/_mapping_registration.cpp
  src/cpp/_perception.cpp
  # disable nanobind using -Os by default, which slows down bound template functions
  NOMINSIZE
  )

target_precompile_headers(_bindings PRIVATE <nanobind/nanobind.h> <nanobind/ndarray.h> <Eigen/Dense>)

# Patch Nanobind nb_func.cpp for better error messages
get_target_property(NBFILES nanobind-static SOURCES)
set(FILTERED_LIST "")
foreach(item IN LISTS NBFILES)
    # If the item does NOT match the pattern, keep it
    if(NOT item MATCHES "nb_func.cpp")  # regex match
        list(APPEND FILTERED_LIST "${item}")
    endif()
endforeach()
set_target_properties(nanobind-static PROPERTIES SOURCES "${FILTERED_LIST};src/cpp/nb_func.cpp")
target_include_directories(nanobind-static PRIVATE "${NB_DIR}/src/")

if(DEFINED OUSTER_SDK_PREBUILT_DIR)
  # Pre-built path: perception is compiled into shared_library.so with hidden
  # visibility.  _bindings only needs the public API.
  #
  # On Apple and Windows, ouster_mapping is absorbed directly into
  # shared_library.{dylib,dll} (see ouster_library/CMakeLists.txt):
  #   - Apple: prevents Ceres/glog from being statically embedded in both the
  #     dylib and _bindings.so, which would cause glog to double-register its
  #     flags at startup.
  #   - Windows: ouster_mapping objects are safe to absorb now that Ceres and
  #     its heavy static deps live in ouster_ceres_deps.dll instead, avoiding
  #     LNK1189 (>65 535 .lib members).
  # On Linux, ouster_mapping is linked separately.
  target_link_libraries(_bindings
    PRIVATE
      OusterSDK::shared_library)
  if(NOT APPLE AND NOT WIN32)
    target_link_libraries(_bindings PRIVATE OusterSDK::ouster_mapping)
  endif()
  if(WIN32)
    # Tell MSVC that OUSTER_API_VAR symbols (e.g. SDK_VERSION, LidarMode
    # statics, enum string tables, viz palettes) come from shared_library.dll
    # so the linker emits __declspec(dllimport) thunks instead of looking for
    # them in static archives (where they don't exist).
    target_compile_definitions(_bindings PRIVATE BUILD_SHARED_LIBS_IMPORT)
  endif()
else()
  # Source wheel: no shared_library; link SDK static targets into _bindings.so.
  # gcc_s before Ceres on Linux to avoid libunwind issues.
  if(UNIX AND NOT APPLE AND TARGET ouster_mapping)
    target_link_libraries(_bindings PRIVATE gcc_s)
  endif()
  target_link_libraries(_bindings
    PRIVATE
      ouster_core
      ouster_sensor
      ouster_build
      ouster_pcap
      ouster_osf
      ouster_viz
      ouster_mapping
      ouster_perception
    )
endif()

# Release build only: silence nanobind shutdown leak diagnostics for end users.
# https://nanobind.readthedocs.io/en/latest/refleaks.html#disabling-leak-warnings
target_compile_definitions(_bindings PRIVATE
  $<$<CONFIG:Release>:SUPPRESS_NANOBIND_LEAK_WARNINGS=1>)

if (TARGET ouster_perception_private)
  library_cleanup_apply_all(_bindings ouster_perception_private)
  library_cleanup_verify(_bindings ouster_perception_private)
endif()
CodeCoverageFunctionality(_bindings)

target_include_directories(_bindings SYSTEM
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/sophus>
    ${CMAKE_CURRENT_LIST_DIR}/src/cpp
    ${numpy_INCLUDE_PATH}
  )

if(NOT MSVC)
  target_compile_options(_bindings PRIVATE -Wno-deprecated-declarations)
endif()

set_target_properties(_bindings PROPERTIES
  POSITION_INDEPENDENT_CODE TRUE
  LIBRARY_OUTPUT_DIRECTORY ${EXT_DIR}/$<0:>)

# Prebuild mode: bundle the shared library alongside _bindings so the wheel is
# self-contained.  Each platform needs different treatment:
#
#   Linux  — auditwheel repair (run post-build in CI) copies libshared_library.so
#             into a .libs/ dir and rewrites RUNPATH.  Nothing needed here.
#
#   macOS  — No post-build repair tool in the pipeline.  We set BUILD_RPATH to
#             @loader_path (the directory containing _bindings.so at load time).
#             The dylib is copied alongside _bindings.so by POST_BUILD so that
#             @loader_path resolves correctly both during local dev and after
#             pip install.  We intentionally do NOT include the prebuilt install
#             dir as an absolute RPATH: that would embed a build-machine path
#             into the wheel and break it on every other machine.
#
#   Windows — No RPATH concept; the DLL loader searches the directory that
#             contains the importing module first, so copying shared_library.dll
#             and ouster_ceres_deps.dll next to _bindings.pyd is sufficient.
#             Ceres and its transitive static deps (glog, gflags, …) are
#             pre-linked into ouster_ceres_deps.dll to avoid LNK1189
#             (>65 535 .lib members) when building shared_library.dll.
if(APPLE AND DEFINED OUSTER_SDK_PREBUILT_DIR)
  set_target_properties(_bindings PROPERTIES
    BUILD_RPATH "@loader_path")
  add_custom_command(TARGET _bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${OUSTER_SDK_PREBUILT_DIR}/lib/libshared_library.dylib"
      "${EXT_DIR}/libshared_library.dylib"
    COMMENT "Bundling libshared_library.dylib into wheel package (macOS)"
    VERBATIM
  )
endif()

if(WIN32 AND DEFINED OUSTER_SDK_PREBUILT_DIR)
  add_custom_command(TARGET _bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${OUSTER_SDK_PREBUILT_DIR}/lib/shared_library.dll"
      "${EXT_DIR}/shared_library.dll"
    COMMENT "Bundling shared_library.dll into wheel package (Windows)"
    VERBATIM
  )
  # ouster_ceres_deps.dll must be shipped alongside shared_library.dll so the
  # Windows DLL loader can resolve Ceres/glog/gflags symbols at runtime.
  add_custom_command(TARGET _bindings POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "${OUSTER_SDK_PREBUILT_DIR}/lib/ouster_ceres_deps.dll"
      "${EXT_DIR}/ouster_ceres_deps.dll"
    COMMENT "Bundling ouster_ceres_deps.dll into wheel package (Windows)"
    VERBATIM
  )
endif()

# Custom nanobind stubgen to fix buggy behavior
function (nanobind_add_stub2 name)
  cmake_parse_arguments(PARSE_ARGV 1 ARG "VERBOSE;INCLUDE_PRIVATE;EXCLUDE_DOCSTRINGS;INSTALL_TIME;RECURSIVE;EXCLUDE_FROM_ALL" "MODULE;COMPONENT;PATTERN_FILE;OUTPUT_PATH" "PYTHON_PATH;DEPENDS;MARKER_FILE;OUTPUT")

  if (EXISTS ${NB_DIR}/src/stubgen.py)
    set(NB_STUBGEN "${CMAKE_CURRENT_LIST_DIR}/src/stubgen.py")
  elseif (EXISTS ${NB_DIR}/stubgen.py)
    set(NB_STUBGEN "${CMAKE_CURRENT_LIST_DIR}/stubgen.py")
  else()
    message(FATAL_ERROR "nanobind_add_stub(): could not locate 'stubgen.py'!")
  endif()

  if (NOT ARG_VERBOSE)
    list(APPEND NB_STUBGEN_ARGS -q)
  else()
    set(NB_STUBGEN_EXTRA USES_TERMINAL)
  endif()

  if (ARG_INCLUDE_PRIVATE)
    list(APPEND NB_STUBGEN_ARGS -P)
  endif()

  if (ARG_EXCLUDE_DOCSTRINGS)
    list(APPEND NB_STUBGEN_ARGS -D)
  endif()

  if (ARG_RECURSIVE)
    list(APPEND NB_STUBGEN_ARGS -r)
  endif()

  foreach (PYTHON_PATH IN LISTS ARG_PYTHON_PATH)
    list(APPEND NB_STUBGEN_ARGS -i "${PYTHON_PATH}")
  endforeach()

  if (ARG_PATTERN_FILE)
    list(APPEND NB_STUBGEN_ARGS -p "${ARG_PATTERN_FILE}")
  endif()

  if (ARG_MARKER_FILE)
    foreach (MARKER_FILE IN LISTS ARG_MARKER_FILE)
      list(APPEND NB_STUBGEN_ARGS -M "${MARKER_FILE}")
      list(APPEND NB_STUBGEN_OUTPUTS "${MARKER_FILE}")
    endforeach()
  endif()

  if (NOT ARG_MODULE)
    message(FATAL_ERROR "nanobind_add_stub(): a 'MODULE' argument must be specified!")
  else()
    list(APPEND NB_STUBGEN_ARGS -m "${ARG_MODULE}")
  endif()

  list(LENGTH ARG_OUTPUT OUTPUT_LEN)

  # Some sanity hecks
  if (ARG_RECURSIVE)
    if (NOT ARG_INSTALL_TIME)
      if ((OUTPUT_LEN EQUAL 0) AND NOT ARG_OUTPUT_PATH)
        message(FATAL_ERROR "nanobind_add_stub(): either 'OUTPUT' or 'OUTPUT_PATH' must be specified when 'RECURSIVE' is set!")
      endif()
    endif()
  else()
    if ((OUTPUT_LEN EQUAL 0) AND NOT ARG_INSTALL_TIME)
      message(FATAL_ERROR "nanobind_add_stub(): an 'OUTPUT' argument must be specified.")
    endif()
    if ((OUTPUT_LEN GREATER 0) AND ARG_OUTPUT_PATH)
      message(FATAL_ERROR "nanobind_add_stub(): 'OUTPUT' and 'OUTPUT_PATH' can only be specified together when 'RECURSIVE' is set!")
    endif()
    if (OUTPUT_LEN GREATER 1)
      message(FATAL_ERROR "nanobind_add_stub(): specifying more than one 'OUTPUT' requires that 'RECURSIVE' is set!")
    endif()
  endif()

  if (ARG_OUTPUT_PATH)
    list(APPEND NB_STUBGEN_ARGS -O "${ARG_OUTPUT_PATH}")
  endif()

  foreach (OUTPUT IN LISTS ARG_OUTPUT)
    if (NOT ARG_RECURSIVE)
      list(APPEND NB_STUBGEN_ARGS -o "${OUTPUT}")
    endif()
    list(APPEND NB_STUBGEN_OUTPUTS "${OUTPUT}")
  endforeach()

  file(TO_CMAKE_PATH ${Python_EXECUTABLE} NB_Python_EXECUTABLE)

  # OUSTER_SDK_GENERATING_STUBS tells the Python package to skip installing the
  # deprecated-kwarg wrappers (see _kwarg_aliases.py), which would otherwise
  # replace typed nanobind signatures with untyped (*args, **kwargs) shims and
  # corrupt the generated .pyi stubs.
  set(NB_STUBGEN_CMD ${CMAKE_COMMAND} -E env OUSTER_SDK_GENERATING_STUBS=1 "${NB_Python_EXECUTABLE}" "${NB_STUBGEN}" ${NB_STUBGEN_ARGS})

  if (NOT WIN32)
    # Pass sanitizer flags to nanobind if needed
    nanobind_sanitizer_preload_env(NB_STUBGEN_ENV ${ARG_DEPENDS})
    if (NB_STUBGEN_ENV)
      nanobind_resolve_python_path()
      if (NB_STUBGEN_ENV MATCHES asan)
        list(APPEND NB_STUBGEN_ENV "ASAN_OPTIONS=detect_leaks=0")
      endif()
      set(NB_STUBGEN_CMD ${CMAKE_COMMAND} -E env OUSTER_SDK_GENERATING_STUBS=1 "${NB_STUBGEN_ENV}" "${NB_PY_PATH}" "${NB_STUBGEN}" ${NB_STUBGEN_ARGS})
    endif()
  endif()

  if (NOT ARG_INSTALL_TIME)
    add_custom_command(
      OUTPUT ${NB_STUBGEN_OUTPUTS}
      COMMAND ${NB_STUBGEN_CMD}
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      DEPENDS ${ARG_DEPENDS} "${NB_STUBGEN}" "${ARG_PATTERN_FILE}"
      ${NB_STUBGEN_EXTRA}
      USES_TERMINAL
    )
    add_custom_target(${name} ALL DEPENDS ${NB_STUBGEN_OUTPUTS})
  else()
    set(NB_STUBGEN_EXTRA "")
    if (ARG_COMPONENT)
      list(APPEND NB_STUBGEN_EXTRA COMPONENT ${ARG_COMPONENT})
    endif()
    if (ARG_EXCLUDE_FROM_ALL)
      list(APPEND NB_STUBGEN_EXTRA EXCLUDE_FROM_ALL)
    endif()
    # \${CMAKE_INSTALL_PREFIX} has same effect as $<INSTALL_PREFIX>
    # This is for compatibility with CMake < 3.27.
    # For more info: https://github.com/wjakob/nanobind/issues/420#issuecomment-1971353531
    install(CODE "set(CMD \"${NB_STUBGEN_CMD}\")\nexecute_process(\n COMMAND \$\{CMD\}\n WORKING_DIRECTORY \"\${CMAKE_INSTALL_PREFIX}\"\n)" ${NB_STUBGEN_EXTRA})
  endif()
endfunction()

nanobind_add_stub2(
  _bindings_stub
  MODULE ouster.sdk._bindings
  PYTHON_PATH ${CMAKE_CURRENT_LIST_DIR}/src;$<TARGET_FILE_DIR:_bindings>/../..
  OUTPUT ouster/sdk/_bindings/client.pyi
  OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/src
  PATTERN_FILE ${CMAKE_CURRENT_LIST_DIR}/stubgen_pattern.txt
  DEPENDS _bindings
  RECURSIVE
  INCLUDE_PRIVATE
)
