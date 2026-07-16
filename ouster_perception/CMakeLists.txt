add_library(ouster_perception_headers INTERFACE)
target_include_directories(ouster_perception_headers
  INTERFACE
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(ouster_perception_headers INTERFACE OusterSDK::ouster_core)

add_library(ouster_perception_stub STATIC src/create_engine_stub.cpp)
set_target_properties(ouster_perception_stub PROPERTIES CXX_STANDARD 14)
target_link_libraries(ouster_perception_stub PUBLIC ouster_perception_headers)

add_library(ouster_perception INTERFACE)
set_target_properties(ouster_perception PROPERTIES CXX_STANDARD 14)
target_link_libraries(ouster_perception INTERFACE
  OusterSDK::ouster_algorithm
  ouster_perception_headers
)

if (TARGET ouster_perception_private)
  message(STATUS "ouster_perception: linking private perception implementation.")
  target_link_libraries(ouster_perception INTERFACE ouster_perception_private)
else()
  message(STATUS "ouster_perception: linking perception stub "
                 "(BUILD_PERCEPTION off or unavailable).")
  target_link_libraries(ouster_perception INTERFACE ouster_perception_stub)
endif()

add_library(OusterSDK::ouster_perception ALIAS ouster_perception)

if(BUILD_SHARED_LIBRARY)
  set_target_properties(ouster_perception PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON)
endif()

if(BUILD_SHARED_LIBRARY)
  install(DIRECTORY include/ouster DESTINATION include)
endif()
