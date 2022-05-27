# Attempt to deal with gtest cmake differences across platforms

function(find_gtest)
  # use system find module in this scope
  set(CMAKE_MODULE_PATH "")

  # using the cmake-provided find module succeeds, but the resulting GTest::GTest and
  # GTest::Main targets cause link errors with vcpkg 2021.05.12 on macos. Try CONFIG-only first
  find_package(GTest CONFIG QUIET)

  if (GTest_CONFIG AND TARGET GTest::gtest AND TARGET GTest::gtest_main)
    find_package_handle_standard_args(GTest DEFAULT_MSG GTest_CONFIG)
    return()
  endif()

  # next, try using a find module. Requires temporarily turning off REQUIRED to avoid failing at the
  # call to find_package()
  set(GTest_FIND_REQUIRED_SAVED ${GTest_FIND_REQUIRED})
  set(GTest_FIND_REQUIRED 0)
  find_package(GTest MODULE QUIET)
  set(GTest_FIND_REQUIRED ${GTest_FIND_REQUIRED_SAVED})

  if (GTEST_FOUND)
    find_package_handle_standard_args(GTest DEFAULT_MSG GTEST_LIBRARY GTEST_MAIN_LIBRARY)

    # aliases to cmake >= 3.20 targets. More portable than IMPORTED_GLOBAL
    # https://cmake.org/cmake/help/v3.22/module/FindGTest.html#imported-targets
    if(NOT TARGET GTest::gtest)
      add_library(GTest::gtest INTERFACE IMPORTED)
      target_link_libraries(GTest::gtest INTERFACE GTest::GTest)
    endif()
    if(NOT TARGET GTest::gtest_main)
      add_library(GTest::gtest_main INTERFACE IMPORTED)
      target_link_libraries(GTest::gtest_main INTERFACE GTest::Main)
    endif()

    return()
  endif()

  # finally, try src location for libgtest-dev for debian-based distros where
  # the find module appears to be broken (xenial, bionic)
  find_path(GTEST_ROOT CMakeLists.txt "/usr/src/gtest")
  find_package_handle_standard_args(GTest DEFAULT_MSG GTEST_ROOT)
  add_subdirectory(${GTEST_ROOT} gtest)
  add_library(GTest::gtest ALIAS gtest)
  add_library(GTest::gtest_main ALIAS gtest_main)

endfunction()

find_gtest()
