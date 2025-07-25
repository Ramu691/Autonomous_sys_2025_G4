cmake_minimum_required(VERSION 3.0.2)
project(detection_pkg)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  tf2
  tf2_ros
)

## Find additional libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

## Declare a catkin package

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs sensor_msgs cv_bridge tf2 tf2_ros
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Add executables
add_executable(lantern_detector src/lantern_detector.cpp)

## Link libraries for lantern_detector
target_link_libraries(lantern_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

## Install targets
install(TARGETS lantern_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
