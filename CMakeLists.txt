cmake_minimum_required(VERSION 2.8)
project(point_cloud_registration)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Werror")

add_compile_definitions(PUBLIC _LIBCPP_NO_EXPERIMENTAL_DEPRECATION_WARNING_FILESYSTEM=1)

# workaround for vtk cmake setup issues on Mac
# include_directories(SYSTEM "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/include/c++/v1")

find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(SOURCES
  src/depth_frame.cpp
  src/depth_pose_graph_optimizer.cpp
  src/depth_to_point_cloud.cpp
  src/feature_map_vertices.cpp
  src/point_cloud_viewer.cpp
)

add_library(${PROJECT_NAME}
  ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${gflags_INCLUDE_DIR}
  ${OpenCV_LIBS}
)

add_executable(load_from_ethdataset
  src/load_from_ethdataset.cpp
)

add_executable(load_from_tum
  src/load_from_tum.cpp
)

target_link_libraries(load_from_ethdataset
  ${PROJECT_NAME}
)

target_link_libraries(load_from_tum
  ${PROJECT_NAME}
)
