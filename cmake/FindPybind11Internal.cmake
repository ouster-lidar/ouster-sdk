include(FindPackageHandleStandardArgs)

if(DEFINED PYTHON_EXECUTABLE)
  execute_process(
    COMMAND
    "${PYTHON_EXECUTABLE}" "-c"
    "import sys; print(f'{str(sys.version_info.major)}.{str(sys.version_info.minor)}.{str(sys.version_info.micro)}')"
    OUTPUT_VARIABLE _version_full
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  execute_process(
    COMMAND
    "${PYTHON_EXECUTABLE}" "-c"
    "import pybind11;print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE _pybind_dir
    RESULT_VARIABLE _pybind_result
    ERROR_VARIABLE _pybind_error
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(${_pybind_result} EQUAL 0)
    LIST(APPEND CMAKE_PREFIX_PATH "${_pybind_dir}")
    if("${_version_full}" VERSION_GREATER_EQUAL "3.11.0")
      find_package(pybind11 2.10 REQUIRED)
    else()
      find_package(pybind11 2.0 REQUIRED)
    endif()

    set(_PYBIND11_INTERNAL_PYTHON_VERSION "")
    if(NOT PYTHON_VERSION STREQUAL "")
      message("Found Python Version VIA: PYTHON_VERSION")
      set(_PYBIND11_INTERNAL_PYTHON_VERSION "${PYTHON_VERSION}")
    elseif(NOT PYTHONLIBS_VERSION_STRING STREQUAL "")
      message("Found Python Version VIA: PYTHONLIBS_VERSION_STRING")
     set(_PYBIND11_INTERNAL_PYTHON_VERSION "${PYTHONLIBS_VERSION_STRING}")
    elseif(NOT PYTHON_VERSION_STRING STREQUAL "")
      message("Found Python Version VIA: PYTHON_VERSION_STRING")
      set(_PYBIND11_INTERNAL_PYTHON_VERSION "${PYTHON_VERSION_STRING}")
    endif()
    if(VCPKG_TOOLCHAIN AND NOT "${_version_full}" VERSION_EQUAL "${_PYBIND11_INTERNAL_PYTHON_VERSION}")
      message(FATAL_ERROR "Python Versions Do Not Match
\tRequested Version:
\t\t${_version_full}
\tVersions Found:
\t\tPYTHON_VERSION: \"${PYTHON_VERSION}\"
\t\tPYTHONLIBS_VERSION_STRING: \"${PYTHONLIBS_VERSION_STRING}\"
\t\tPYTHON_VERSION_STRING: \"${PYTHON_VERSION_STRING}\"
\tInternal Cache: \"${_PYBIND11_INTERNAL_PYTHON_VERSION}\"")
      endif()
  else()
    message(FATAL_ERROR "ERROR In Setting Pybind11 CMAKE Prefix Path: ${_pybind_error}")
  endif()
else()
  message(FATAL_ERROR "PYTHON_EXECUTABLE NOT SET")
endif()

