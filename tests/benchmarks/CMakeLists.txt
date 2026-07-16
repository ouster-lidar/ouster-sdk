# Benchmark targets — built whenever BUILD_TESTING=ON
# By default each benchmark runs 1 iteration as a smoke test.
# Set OUSTER_PERFORMANCE_BENCHMARK=1 to run all iterations.

function(ouster_benchmark target_name)
  add_executable(${target_name} ${ARGN})
  target_include_directories(${target_name}
    PRIVATE
      ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/sophus
      ${CMAKE_CURRENT_LIST_DIR}/../../ouster_mapping/src
      ${CMAKE_CURRENT_LIST_DIR}/../../ouster_core/include
  )
  if(BUILD_MAPPING)
    target_include_directories(${target_name}
      PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/../../thirdparty/robin-map/include)
  endif()
  target_compile_features(${target_name} PRIVATE cxx_std_17)
  target_link_libraries(${target_name}
    PRIVATE
      ${LIB_TO_USE}
      GTest::gtest
      GTest::gtest_main
  )
  if(BUILD_MAPPING AND TARGET tsl::robin_map)
    target_link_libraries(${target_name} PRIVATE tsl::robin_map)
  endif()
  add_test(
    NAME ${target_name}
    COMMAND ${target_name} --gtest_output=xml:${target_name}.xml
  )
  set_tests_properties(${target_name}
    PROPERTIES
      LABELS "benchmark"
  )
endfunction()


if(BUILD_MAPPING)
  ouster_benchmark(mapping_benchmark
  ${CMAKE_CURRENT_LIST_DIR}/mapping_benchmark.cpp)
endif()

if(BUILD_PCAP)
  ouster_benchmark(core_benchmark
  ${CMAKE_CURRENT_LIST_DIR}/core_benchmark.cpp)
endif()
