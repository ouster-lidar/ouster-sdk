cmake_minimum_required(VERSION 3.1.0)

# ==== Project Name ====
project(ouster_client)

# ==== Requirements ====
SET(OUSTER_CLIENT_ADDITIONAL_LIBS "")
SET(OUSTER_CLIENT_ADDITIONAL_LIB_DIRS "")
IF (WIN32)
	find_package(jsoncpp CONFIG REQUIRED)
	SET(OUSTER_CLIENT_ADDITIONAL_LIBS
		jsoncpp
		ws2_32)
	get_target_property(jsoncpp_INCLUDE_DIRS jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
	SET(OUSTER_CLIENT_ADDITIONAL_LIB_DIRS ${_VCPKG_INSTALLED_DIR}/x64-windows-static/lib)
ELSE()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(jsoncpp REQUIRED jsoncpp)
  SET(OUSTER_CLIENT_ADDITIONAL_LIBS
    jsoncpp)
ENDIF()

find_package(Eigen3 REQUIRED)

# ==== Options ====
set(CMAKE_CXX_STANDARD 11)
add_compile_options("-Wall")

# ==== Libraries ====
add_library(ouster_client STATIC src/os1.cpp src/os1_util.cpp src/compat.cpp)
target_link_libraries(ouster_client PUBLIC ${OUSTER_CLIENT_ADDITIONAL_LIBS})
target_include_directories(ouster_client PUBLIC include ${OUSTER_CLIENT_ADDITIONAL_INCLUDES})
target_include_directories(ouster_client SYSTEM PUBLIC ${jsoncpp_INCLUDE_DIRS} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
IF (WIN32)
  target_link_directories(ouster_client PUBLIC ${OUSTER_CLIENT_ADDITIONAL_LIB_DIRS})
ENDIF()
# ==== Executables ====
add_executable(ouster_client_example src/main.cpp)
target_link_libraries(ouster_client_example ouster_client)
target_include_directories(ouster_client_example PRIVATE include)
