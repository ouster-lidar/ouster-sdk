find_package(Eigen3 REQUIRED)

# On Windows, Ceres and its transitive static libs (glog, gflags, SuiteSparse,
# etc.) can exceed the 65,535-member limit of a MSVC .lib archive (LNK1189)
# when their object files are absorbed into shared_library.dll via
# $<TARGET_OBJECTS:...>.  The fix is to pre-link those heavy third-party deps
# into their own DLL (ouster_ceres_deps.dll) and export all their symbols via
# /WHOLEARCHIVE + WINDOWS_EXPORT_ALL_SYMBOLS.  shared_library.dll then
# resolves Ceres/glog symbols by importing from ouster_ceres_deps.dll rather
# than absorbing the full static archives directly.
if(WIN32 AND BUILD_SHARED_LIBRARY)
  find_package(Ceres REQUIRED)
  add_library(ouster_ceres_deps SHARED empty.cpp)
  target_compile_definitions(ouster_ceres_deps PRIVATE BUILD_OUSTER_CERES_DEPS_DLL)
  set_property(TARGET ouster_ceres_deps PROPERTY POSITION_INDEPENDENT_CODE ON)
  # WINDOWS_EXPORT_ALL_SYMBOLS generates a .def from the target's own .obj
  # files, but NOT from symbols in statically-linked archives (Ceres, glog,
  # gflags, etc.).  We must force the linker to absorb every object file from
  # those archives (/WHOLEARCHIVE) so that WINDOWS_EXPORT_ALL_SYMBOLS picks
  # them up and the resulting DLL's export table contains those symbols.
  # Without this, shared_library.dll cannot resolve ceres::Problem and
  # google::Log* symbols at link time even though ouster_ceres_deps.dll is
  # listed as a dependency.
  set_property(TARGET ouster_ceres_deps PROPERTY WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # WHOLE_ARCHIVE does not propagate transitively through CMake's dependency
  # graph — it must be applied to each static lib individually.  Walk the
  # full transitive INTERFACE_LINK_LIBRARIES of Ceres::ceres to collect every
  # STATIC_LIBRARY target, then apply WHOLE_ARCHIVE to each so their symbols
  # are absorbed into (and exported from) ouster_ceres_deps.dll.
  #
  # BFS over the dependency graph.  list(POP_FRONT) requires CMake 3.15; we
  # use list(GET)/list(REMOVE_AT) instead for compatibility back to 3.10.
  set(_ceres_worklist "Ceres::ceres")
  set(_ceres_visited "")
  set(_ceres_static_deps "")
  while(_ceres_worklist)
    list(GET _ceres_worklist 0 _cur)
    list(REMOVE_AT _ceres_worklist 0)
    list(FIND _ceres_visited "${_cur}" _seen)
    if(_seen EQUAL -1)
      list(APPEND _ceres_visited "${_cur}")
      if(TARGET "${_cur}")
        get_target_property(_cur_type "${_cur}" TYPE)
        if(_cur_type STREQUAL "STATIC_LIBRARY")
          list(APPEND _ceres_static_deps "${_cur}")
        endif()
        get_target_property(_cur_iface "${_cur}" INTERFACE_LINK_LIBRARIES)
        if(_cur_iface)
          foreach(_lib ${_cur_iface})
            # Skip generator expressions — only plain target names are walkable.
            if(NOT _lib MATCHES "^\\$<")
              list(APPEND _ceres_worklist "${_lib}")
            endif()
          endforeach()
        endif()
      endif()
    endif()
  endwhile()

  target_link_libraries(ouster_ceres_deps PUBLIC Ceres::ceres)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    foreach(_dep ${_ceres_static_deps})
      target_link_libraries(ouster_ceres_deps PRIVATE
        "$<LINK_LIBRARY:WHOLE_ARCHIVE,${_dep}>")
    endforeach()
  else()
    foreach(_dep ${_ceres_static_deps})
      target_link_options(ouster_ceres_deps PRIVATE
        "/WHOLEARCHIVE:$<TARGET_FILE:${_dep}>")
    endforeach()
  endif()
  add_library(OusterSDK::ouster_ceres_deps ALIAS ouster_ceres_deps)
  install(TARGETS ouster_ceres_deps
    EXPORT ouster-sdk-targets
    RUNTIME DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include)
endif()

function(ouster_library_common _TARGET)
  message("Building Library: ${_TARGET}")
  target_link_libraries(${_TARGET}
    PUBLIC
    Eigen3::Eigen)
  target_include_directories(${_TARGET} SYSTEM
    PUBLIC
    $<INSTALL_INTERFACE:include/optional-lite>)
  get_target_property(CLIENT_INCLUDE_DIRS ouster_core INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(SENSOR_INCLUDE_DIRS ouster_sensor INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(${_TARGET}
    PUBLIC
    $<BUILD_INTERFACE:${CLIENT_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${SENSOR_INCLUDE_DIRS}>
    $<INSTALL_INTERFACE:include>)

  set_property(TARGET ${_TARGET} PROPERTY POSITION_INDEPENDENT_CODE ON)
  set_target_properties(${_TARGET} PROPERTIES CXX_VISIBILITY_PRESET hidden)
  target_link_libraries(${_TARGET} PRIVATE ouster_core ouster_sensor)
  # ouster_core is also absorbed via $<TARGET_OBJECTS:ouster_core> into
  # shared_library; so transitive libs like ouster_build (SDK_VERSION, BUILD_HASH,
  # etc.) are never pulled unless linked explicitly.
  if(TARGET ouster_build)
    target_link_libraries(${_TARGET} PRIVATE ouster_build)
  endif()
  message("Building Library: ${_TARGET}: Adding ouster_core")
  message("Building Library: ${_TARGET}: CLIENT_INCLUDE_DIRS: ${CLIENT_INCLUDE_DIRS}")
  message("Building Library: ${_TARGET}: SENSOR_INCLUDE_DIRS: ${SENSOR_INCLUDE_DIRS}")
  # ouster_mapping is absorbed via $<TARGET_OBJECTS:...> directly into
  # shared_library (see below) on Apple and Windows, rather than being linked
  # as a static archive through ouster_library_common, to give us precise
  # control over how its Ceres dependency is resolved on each platform.
  if(BUILD_PCAP)
    target_link_libraries(${_TARGET} PRIVATE ouster_pcap)
    get_target_property(_INCLUDE_DIRS ouster_pcap INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${_TARGET}
      PUBLIC
      $<BUILD_INTERFACE:${_INCLUDE_DIRS}>)
    message("Building Library: ${_TARGET}: Adding ouster_pcap")
    message("Building Library: ${_TARGET}: PCAP_INCLUDE_DIRS: ${_INCLUDE_DIRS}")
  endif()
  if(BUILD_OSF)
    target_link_libraries(${_TARGET} PRIVATE ouster_osf)
    get_target_property(_INCLUDE_DIRS ouster_osf INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${_TARGET}
      PUBLIC
      $<BUILD_INTERFACE:${_INCLUDE_DIRS}>)
    message("Building Library: ${_TARGET}: Adding ouster_osf")
    message("Building Library: ${_TARGET}: OSF_INCLUDE_DIRS: ${_INCLUDE_DIRS}")
  endif()
  if(BUILD_VIZ)
    target_link_libraries(${_TARGET} PRIVATE ouster_viz)
    get_target_property(_INCLUDE_DIRS ouster_viz INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${_TARGET}
      PUBLIC
      $<BUILD_INTERFACE:${_INCLUDE_DIRS}>)
    message("Building Library: ${_TARGET}: Adding ouster_viz")
    message("Building Library: ${_TARGET}: VIZ_INCLUDE_DIRS: ${_INCLUDE_DIRS}")
  endif()

  add_library(OusterSDK::${_TARGET} ALIAS ${_TARGET})
endfunction()

if(BUILD_SHARED_LIBRARY)
  add_library(shared_library SHARED
    $<TARGET_OBJECTS:ouster_core>
    $<TARGET_OBJECTS:ouster_algorithm>
    $<TARGET_OBJECTS:ouster_build>)
  get_target_property(_ALGORITHM_INCLUDE_DIRS ouster_algorithm
    INTERFACE_INCLUDE_DIRECTORIES)
  if(_ALGORITHM_INCLUDE_DIRS)
    target_include_directories(shared_library PUBLIC
      $<BUILD_INTERFACE:${_ALGORITHM_INCLUDE_DIRS}>)
  endif()
  if(TARGET ouster_pcap)
      target_sources(shared_library PRIVATE $<TARGET_OBJECTS:ouster_pcap>)
  endif()
  if(TARGET ouster_osf)
      target_sources(shared_library PRIVATE $<TARGET_OBJECTS:ouster_osf>)
  endif()
  if(TARGET ouster_viz)
      target_sources(shared_library PRIVATE $<TARGET_OBJECTS:ouster_viz>)
  endif()
  if(TARGET ouster_sensor)
    target_sources(shared_library PRIVATE $<TARGET_OBJECTS:ouster_sensor>)
  endif()

  if(TARGET ouster_perception AND TARGET ouster_perception_private)
    target_sources(shared_library PRIVATE
      $<TARGET_OBJECTS:ouster_perception_private>
    )
  elseif(TARGET ouster_perception AND WIN32)
    # Stub build (no ouster_perception_private): absorb ouster_perception and
    # ouster_perception_stub so their objects are compiled with
    # BUILD_SHARED_LIBS_EXPORT and OUSTER_API_FUNCTION symbols such as
    # PerceptionEngine::create are exported from shared_library.dll.
    target_sources(shared_library PRIVATE
      $<TARGET_OBJECTS:ouster_perception_stub>)
    get_target_property(_PERC_INCLUDE_DIRS ouster_perception_headers
      INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(shared_library PUBLIC
      $<BUILD_INTERFACE:${_PERC_INCLUDE_DIRS}>)
  endif()
  if(TARGET ouster_perception AND TARGET ouster_perception_private)
    # External deps whose symbols are pulled in by ouster_perception_private
    # objects but are not already in the shared library.
    # On Windows we link ouster_ceres_deps (a DLL) instead of the static Ceres
    # archives to avoid LNK1189.  On all other platforms we link Ceres directly.
    if(WIN32)
      target_link_libraries(shared_library PRIVATE OusterSDK::ouster_ceres_deps)
    else()
      find_package(Ceres REQUIRED)
      target_link_libraries(shared_library PRIVATE Ceres::ceres)
    endif()
    find_package(CGAL 5.4 REQUIRED)
    target_link_libraries(shared_library PRIVATE CGAL::CGAL)
    if(UNIX AND NOT APPLE)
      target_link_libraries(shared_library PRIVATE gcc_s)
    endif()
    # Expose public perception headers through the shared library's interface.
    get_target_property(_PERC_INCLUDE_DIRS ouster_perception_headers
      INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(shared_library PUBLIC
      $<BUILD_INTERFACE:${_PERC_INCLUDE_DIRS}>)
    message("Building Library: shared_library: Adding ouster_perception")
  endif()

  # Absorb ouster_mapping objects into shared_library on Apple, Windows, and Linux.
  # - Apple: avoids Ceres/glog being in both dylib and _bindings (double flag init).
  # - Windows: safe with ouster_ceres_deps DLL; avoids LNK1189 from static Ceres.
  # - Linux: same glog/Ceres issue as Apple if _bindings also links ouster_mapping;
  #   absorb here so the extension only links shared_library (see python/CMakeLists).
  if(TARGET ouster_mapping AND (UNIX OR WIN32))
    target_sources(shared_library PRIVATE $<TARGET_OBJECTS:ouster_mapping>)
    if(WIN32)
      # Link ouster_ceres_deps unconditionally here — the perception block
      # above also links it, but perception may be absent (BUILD_PERCEPTION=OFF
      # means ouster_perception_private never exists and that if() is false).
      # CMake deduplicates repeated entries in the link line, so linking it
      # twice is harmless.
      target_link_libraries(shared_library PRIVATE OusterSDK::ouster_ceres_deps)
    else()
      # Apple path — Ceres was already found and linked above (perception block);
      # guard against the case where perception is absent so the build still works.
      if(NOT TARGET Ceres::ceres)
        find_package(Ceres QUIET)
      endif()
      if(TARGET Ceres::ceres)
        target_link_libraries(shared_library PRIVATE Ceres::ceres)
      endif()
      # TBB is used directly by icp_registration.cpp (absorbed from kiss-icp into
      # ouster_mapping). It was previously a transitive dep through kiss_icp_core;
      # now it must be linked explicitly when absorbing ouster_mapping objects.
      find_package(OpenMP QUIET)
      if(TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(shared_library PRIVATE OpenMP::OpenMP_CXX)
      endif()
    endif()
    find_package(TBB CONFIG QUIET)
    if(TARGET TBB::tbb)
      target_link_libraries(shared_library PRIVATE TBB::tbb)
    endif()
    get_target_property(_MAPPING_INCLUDE_DIRS ouster_mapping
      INTERFACE_INCLUDE_DIRECTORIES)
    if(_MAPPING_INCLUDE_DIRS)
      target_include_directories(shared_library PUBLIC
        $<BUILD_INTERFACE:${_MAPPING_INCLUDE_DIRS}>)
    endif()
    if(WIN32)
      message("Building Library: shared_library: Absorbing ouster_mapping (Windows)")
    else()
      message("Building Library: shared_library: Absorbing ouster_mapping (Unix)")
    endif()
  endif()

  ouster_library_common(shared_library)
  target_compile_definitions(shared_library PRIVATE BUILD_SHARED_LIBS_EXPORT)
  # On Windows, OUSTER_API_FUNCTION expands to __declspec(dllexport) only when
  # BUILD_SHARED_LIBS_EXPORT is defined at compile time.  The object files
  # absorbed via $<TARGET_OBJECTS:...> are compiled as part of their own static
  # library targets, not as part of shared_library, so the PRIVATE definition
  # above does not reach them.  Propagate the define to each absorbed target so
  # their objects are compiled with dllexport and the linker emits the symbols
  # into shared_library.dll's export table.
  if(WIN32)
    # Propagate BUILD_SHARED_LIBS_EXPORT to each public absorbed target so their
    # objects are compiled with __declspec(dllexport) on OUSTER_API_FUNCTION
    # symbols and the linker emits them into shared_library.dll's export table.
    #
    # ouster_perception_private is included so the public perception API
    # (DetectionEngine::create and friends, all OUSTER_API_FUNCTION-annotated in
    # detection_engine.h) is exported; otherwise _bindings.pyd fails to link with
    # LNK2019 on those symbols.  shared_library does NOT use
    # WINDOWS_EXPORT_ALL_SYMBOLS (that is only set on ouster_ceres_deps), so
    # exports come solely from __declspec(dllexport) — only the annotated public
    # symbols are exported.  The private scrambled classes (PointCloud,
    # ClassicEngine, etc.) carry no OUSTER_API_FUNCTION annotation and therefore
    # stay out of the export table; the symbol_scrambler check_symbols gate
    # verifies this and will fail loudly if any private symbol ever leaks.
    foreach(_absorbed_target
        ouster_build ouster_core ouster_algorithm ouster_pcap ouster_osf
        ouster_viz ouster_sensor ouster_mapping ouster_perception_stub
        ouster_perception_private)
      if(TARGET ${_absorbed_target})
        target_compile_definitions(${_absorbed_target} PRIVATE BUILD_SHARED_LIBS_EXPORT)
      endif()
    endforeach()
  endif()

  if(TARGET ouster_perception_private)
    library_cleanup_apply_flags(shared_library)
  endif()

  if(TARGET ouster_perception_private)
    library_cleanup_verify(shared_library ouster_perception_private)
  endif()
endif()

if(BUILD_SHARED_LIBRARY)
  install(TARGETS shared_library
        EXPORT ouster-sdk-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include)
endif()
