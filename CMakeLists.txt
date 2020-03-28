cmake_minimum_required(VERSION 2.8)
project(point_cloud_registration)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-g3")

# workaround for vtk cmake setup issues on Mac
include_directories(SYSTEM "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/include/c++/v1")

find_package(gflags REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4.0 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(load_from_ethdataset src/load_from_ethdataset.cpp)
target_link_libraries(load_from_ethdataset
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${gflags_INCLUDE_DIR}
  ${OpenCV_LIBS}
)

add_executable(load_from_tum src/load_from_tum.cpp)
target_link_libraries(load_from_tum
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${gflags_INCLUDE_DIR}
  ${OpenCV_LIBS}
)
