find_package(Eigen3 REQUIRED)

function(ouster_library_common _TARGET)
  message("Building Library: ${_TARGET}")
  target_link_libraries(${_TARGET}
    PUBLIC
    Eigen3::Eigen)
  target_include_directories(${_TARGET} SYSTEM
    PUBLIC
    $<INSTALL_INTERFACE:include/optional-lite>)
  get_target_property(CLIENT_INCLUDE_DIRS ouster_client INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(SENSOR_INCLUDE_DIRS ouster_sensor INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(${_TARGET}
    PUBLIC
    $<BUILD_INTERFACE:${CLIENT_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${SENSOR_INCLUDE_DIRS}>
	$<INSTALL_INTERFACE:include>
	$<INSTALL_INTERFACE:include/fb_generated>)

  set_property(TARGET ${_TARGET} PROPERTY POSITION_INDEPENDENT_CODE ON)
  set_target_properties(${_TARGET} PROPERTIES CXX_VISIBILITY_PRESET hidden)
  target_link_libraries(${_TARGET} PRIVATE ouster_client ouster_sensor)
  message("Building Library: ${_TARGET}: Adding ouster_client")
  if(BUILD_PCAP)
    target_link_libraries(${_TARGET} PRIVATE ouster_pcap)
    get_target_property(_INCLUDE_DIRS ouster_pcap INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${_TARGET}
      PUBLIC
      $<BUILD_INTERFACE:${_INCLUDE_DIRS}>)
    message("Building Library: ${_TARGET}: Adding ouster_pcap")
  endif()
  if(BUILD_OSF)
    target_link_libraries(${_TARGET} PRIVATE ouster_osf)
    get_target_property(_INCLUDE_DIRS ouster_osf INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${_TARGET}
      PUBLIC
      $<BUILD_INTERFACE:${_INCLUDE_DIRS}>)
    message("Building Library: ${_TARGET}: Adding ouster_osf")
  endif()
  if(BUILD_VIZ)
    target_link_libraries(${_TARGET} PRIVATE ouster_viz)
    get_target_property(_INCLUDE_DIRS ouster_viz INTERFACE_INCLUDE_DIRECTORIES)
    target_include_directories(${_TARGET}
      PUBLIC
      $<BUILD_INTERFACE:${_INCLUDE_DIRS}>)
    message("Building Library: ${_TARGET}: Adding ouster_viz")
  endif()

  add_library(OusterSDK::${_TARGET} ALIAS ${_TARGET})
endfunction()

set(OUSTER_LIBRARY_OBJECTS "")
if(BUILD_SHARED_LIBRARY)
  # FIGURE OUT HOW TO SKIP VIZ OSF AND PCAP HERE
  add_library(shared_library SHARED
    $<TARGET_OBJECTS:ouster_client>)
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
  ouster_library_common(shared_library)
  target_compile_definitions(shared_library PRIVATE BUILD_SHARED_LIBS_EXPORT)
endif()

if(BUILD_SHARED_LIBRARY)
  install(TARGETS shared_library
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include)
endif()
