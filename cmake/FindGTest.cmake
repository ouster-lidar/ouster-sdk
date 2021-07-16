function(find_gtest)
  # use system find module in this scope
  set(CMAKE_MODULE_PATH "")

  find_package(GTest QUIET)

  if(GTEST_FOUND)
    find_package_handle_standard_args(GTest DEFAULT_MSG GTEST_LIBRARY)
  else()
    message(STATUS "Looking for GTest source in /usr/src/gtest")
    add_subdirectory("/usr/src/gtest" gtest)
    add_library(GTest::GTest ALIAS gtest)
    add_library(GTest::Main ALIAS gtest_main)
    message(STATUS "Found GTest source")
  endif()

endfunction()

find_gtest()
