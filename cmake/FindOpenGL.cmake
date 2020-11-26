# try to define a forward-compatible imported target on old platforms (Ubunu 16.04)

function(find_opengl)
  set(CMAKE_MODULE_PATH "")

  find_package(OpenGL QUIET REQUIRED)
  find_package_handle_standard_args(OpenGL DEFAULT_MSG OPENGL_LIBRARIES)

  if(NOT TARGET OpenGL::GL)
    if(OPENGL_gl_LIBRARY AND OPENGL_INCLUDE_DIR)
      add_library(OpenGL::GL UNKNOWN IMPORTED)
      set_target_properties(OpenGL::GL PROPERTIES
        IMPORTED_LOCATION "${OPENGL_gl_LIBRARY}")
      set_target_properties(OpenGL::GL PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${OPENGL_INCLUDE_DIR}")
    else()
      message(FATAL_ERROR "Failed to provide required OpenGL::GL target")
    endif()
  endif()

endfunction()

find_opengl()
