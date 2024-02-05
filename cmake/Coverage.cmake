# Cmake Functions For Code Coverage

FUNCTION(CodeCoverageFunctionality target)
  if(DEFINED ENV{CMAKE_COVERAGE_TESTS} AND "$ENV{CMAKE_COVERAGE_TESTS}" MATCHES "true")
    message(STATUS "Code Coverage Enabled: Target: ${target}")
    target_link_libraries(${target} PRIVATE gcov)
    target_compile_options(${target} PRIVATE -O0 --coverage -g)
  endif()
ENDFUNCTION()
