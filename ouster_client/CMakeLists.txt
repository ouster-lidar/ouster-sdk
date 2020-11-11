cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_client)

# ==== Requirements ====
set(OUSTER_CLIENT_ADDITIONAL_LIBS "")
set(OUSTER_CLIENT_ADDITIONAL_LIB_DIRS "")

if(WIN32)
  find_package(jsoncpp CONFIG REQUIRED)
  string(FIND ${CMAKE_CXX_FLAGS} "/MT" IS_MT)
  if(IS_MT EQUAL -1)
    set(OUSTER_CLIENT_ADDITIONAL_LIBS jsoncpp_lib ws2_32)
  else()
    set(OUSTER_CLIENT_ADDITIONAL_LIBS jsoncpp ws2_32)
  endif()

  get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib
                      INTERFACE_INCLUDE_DIRECTORIES)
  set(OUSTER_CLIENT_ADDITIONAL_LIB_DIRS
      ${_VCPKG_INSTALLED_DIR}/x64-windows-static/lib)
  add_definitions(-D_USE_MATH_DEFINES)
elseif(APPLE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(JSONCPP REQUIRED jsoncpp)
  set(OUSTER_CLIENT_ADDITIONAL_INCLUDES ${JSONCPP_INCLUDEDIR})
  set(OUSTER_CLIENT_ADDITIONAL_LIBS ${JSONCPP_LIBRARIES})
  set(OUSTER_CLIENT_ADDITIONAL_LIB_DIRS ${JSONCPP_LIBDIR})
  add_compile_options(-Wextra -Werror)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(jsoncpp REQUIRED jsoncpp)
  set(OUSTER_CLIENT_ADDITIONAL_LIBS jsoncpp)
  add_compile_options(-Wextra -Werror)
endif()

find_package(Eigen3 REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall)

# ==== Libraries ====
add_library(ouster_client STATIC src/client.cpp src/types.cpp src/compat.cpp src/lidar_scan.cpp)
set_target_properties(ouster_client PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(ouster_client PUBLIC ${OUSTER_CLIENT_ADDITIONAL_LIBS})
target_include_directories(ouster_client
                           PUBLIC include ${OUSTER_CLIENT_ADDITIONAL_INCLUDES})
target_include_directories(
  ouster_client SYSTEM
  PUBLIC ${jsoncpp_INCLUDE_DIRS} SYSTEM
  PRIVATE ${EIGEN3_INCLUDE_DIR})
if(WIN32 OR APPLE)
  target_link_directories(ouster_client PUBLIC
                          ${OUSTER_CLIENT_ADDITIONAL_LIB_DIRS})
endif()
# ==== Executables ====
add_executable(ouster_client_example src/main.cpp)
target_link_libraries(ouster_client_example ouster_client)
target_include_directories(ouster_client_example PRIVATE include)
