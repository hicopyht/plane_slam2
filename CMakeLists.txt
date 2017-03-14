cmake_minimum_required(VERSION 2.8.3)
project(plane_slam2)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  line_based_plane_segment
  message_filters
  roscpp
  sensor_msgs
  tf
)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals thread)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Pangolin REQUIRED)

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation
# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
# Find TBB
find_package(TBB)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES plane_slam2
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure line_based_plane_segment message_filters pcl_ros plane_from_line roscpp sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Wrap_INCLUDE_DIR}
  ${CppUnitLite_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
  src/Utils.cc
  src/OrbExtractor.cc
  src/PlaneSegmentor.cc
  src/MapPlane.cc
  src/MapPoint.cc
  src/Map.cc
  src/Frame.cc
  src/Tracking.cc
  src/Mapping.cc
  src/System.cc
  src/Viewer.cc
)

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CppUnitLite_LIBS}
  gtsam
  GL
  line_based_plane_segment
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/ROS/rgbd)
add_executable(rgbd ROS/rgbd/rgbd.cc)
target_link_libraries(rgbd ${PROJECT_NAME} ${catkin_LIBRARIES})
