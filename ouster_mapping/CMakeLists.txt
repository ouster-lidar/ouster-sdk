cmake_minimum_required(VERSION 3.16.3)

option(USE_OPENMP "Enable the use of OpenMP" ON)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(KissIcp REQUIRED)

if(USE_OPENMP)

  if (APPLE)
    message(STATUS "NOTE: On MacOS you may need to install libomp via Homebrew or MacPorts")
    find_package(OPENMP REQUIRED)
  else()
    find_package(OpenMP REQUIRED)
  endif()
  if(OPENMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()
endif()


# Create a shared library for pose_optimizer_py
add_library(ouster_mapping STATIC
    src/pose_optimizer.cpp
    src/utils.cpp
    src/absolute_pose_constraint.cpp
    src/pose_to_pose_constraint.cpp
    src/point_to_point_constraint.cpp
    src/pose_optimizer_node.cpp
    src/trajectory.cpp
    src/preprocessing.cpp
)

target_include_directories(ouster_mapping
   PRIVATE
     ouster_transform
     ${CERES_INCLUDE_DIRS}
     ${EIGEN3_INCLUDE_DIRS}
   PUBLIC
      $<INSTALL_INTERFACE:include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# On Linux gcc_s MUST be linked against before CERES to avoid libunwind issues
# Numpy and friends use gcc_s for exception handling, but CERES (actually its dependency glog) 
# wants to use libunwind. If you mix and match bad things happen.
# Putting gcc_s first causes the linker to prefer it.
if(UNIX AND NOT APPLE)
set(MAPPING_OS_LIBRARIES gcc_s)
else()
set(MAPPING_OS_LIBRARIES)
endif()

set(CERES_LIBRARIES_INTERNAL Ceres::ceres)
if (NOT TARGET Ceres::ceres)
    set(CERES_LIBRARIES_INTERNAL ${CERES_LIBRARIES})
endif()

set(OPENMP_LIBRARIES_INTERNAL "")
if(USE_OPENMP AND OPENMP_CXX_FOUND)
  set(OPENMP_LIBRARIES_INTERNAL OpenMP::OpenMP_CXX)
endif()

target_link_libraries(ouster_mapping
  PRIVATE
    ${MAPPING_OS_LIBRARIES}
    $<BUILD_INTERFACE:kiss_icp_core>
    $<BUILD_INTERFACE:kiss_icp_pipeline>
    ${CERES_LIBRARIES_INTERNAL}
    Eigen3::Eigen
    OusterSDK::ouster_client
    OusterSDK::ouster_osf
    ${OPENMP_LIBRARIES_INTERNAL}
)

target_compile_definitions(ouster_mapping PUBLIC GLOG_USE_GLOG_EXPORT)

if(USE_OPENMP AND OPENMP_CXX_FOUND)
  target_compile_definitions(ouster_mapping PRIVATE OUSTER_OMP)
endif()

add_library(OusterSDK::ouster_mapping ALIAS ouster_mapping)

install(TARGETS ouster_mapping
  EXPORT ouster-sdk-targets
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(DIRECTORY include/ouster DESTINATION include)
